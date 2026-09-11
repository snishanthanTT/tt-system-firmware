/* SMC OCCP Interface Implementation
 * On-Chip Command Processor for I2C/I3C communication
 *
 * Ported from tt_smc firmware/prod_rom/lib/src/occp.c (tt_smc 935b8fe21,
 * 2025-12-29) to run inside the Zephyr dm_test_app on a Nucleo U385 that
 * stands in for a Mimir chiplet SMC. The protocol logic (header and body
 * CRC, command handlers, interface latching, error replies) is the ROM's.
 * What changed:
 *
 *   - Bus access goes through the occp_link_t mailbox in smc_occp_port.h
 *     instead of the Cadence I3C and DesignWare I2C register drivers.
 *   - ReadData and WriteData touch an emulated memory window, not the
 *     SMC address space.
 *   - eFuse, strap, GPIO, scratchpad, POST code, status ring buffer and the
 *     simulation console are constants, logs or stubs from the port layer.
 *   - ExecuteImage and AuthenticateImage answer like the ROM but do not jump
 *     or halt: there is no RISC-V image to run on this board.
 *   - The poll loop sleeps on a semaphore when no command is waiting, so the
 *     shell and logging threads keep running.
 *   - Only one I3C interface is registered. The I2C interface is Phase 6.
 */

#include <stdint.h>
#include <string.h>

#include "smc_occp.h"
#include "smc_occp_error_codes.h"
#include "smc_occp_port.h"
#include "smc_occp_status.h"

LOG_MODULE_DECLARE(occp_tgt, CONFIG_DM_TEST_APP_OCCP_TARGET_LOG_LEVEL);

/* The wire format is little-endian, LSB-first bitfields. Same on Arm as on RISC-V. */
BUILD_ASSERT(sizeof(occp_header) == 4, "occp_header must be 4 bytes");
BUILD_ASSERT(sizeof(packet_header) == 8, "packet_header must be 8 bytes");
BUILD_ASSERT(sizeof(get_version_response) == 13, "get_version_response must be 13 bytes");
BUILD_ASSERT(sizeof(get_status_response) == 13, "get_status_response must be 13 bytes");
BUILD_ASSERT(sizeof(error_response) == 13, "error_response must be 13 bytes");

/*********************************************************************
 * Type Definitions and Constants
 ********************************************************************/

typedef occp_link_t *interface_driver_t;

typedef enum
{
    DRIVER_TYPE_I3C,
    DRIVER_TYPE_I2C
} driver_type_t;

typedef struct
{
    interface_driver_t channel_drivers[5];
    driver_type_t type[5];
    size_t num_channels;

} smc_active_interfaces_t;

static uint32_t TRANSPORT_TIMEOUT = 10000; /* Global transport timeout default value*/
/* The ROM reads this from eFuse. The port keeps the ROM default. */
static void smc_occp_init_transport_timeout(void)
{
    TRANSPORT_TIMEOUT = 10000;
    simputshex32("OCCP: Transport timeout set to: ", TRANSPORT_TIMEOUT);
}

/* How long the poll loop sleeps when no interface has a command. */
#define OCCP_POLL_IDLE_WAIT K_MSEC(10)

/*********************************************************************
 * Function Prototypes
 ********************************************************************/

static uint64_t smc_occp_determine_i3c_address(uint8_t efuse_slot_id);
static int smc_occp_init_i3c_channel(bool use_channel, uint8_t controller_id, uint64_t i3c_id);
static int smc_occp_poll_channels(void);
static void smc_occp_latch_interface(int interface_index);
static void smc_occp_unlatch_interface(void);
static bool smc_occp_interface_has_data(int interface_index);
static bool smc_occp_handle_interface_error(void);
static int smc_occp_send_to_bus(interface_driver_t drv, driver_type_t drv_type, const uint8_t *data, size_t length, uint32_t timeout);
static int smc_occp_read_from_bus_4byte_aligned_or_complete_stream(interface_driver_t drv, driver_type_t drv_type, uint8_t *buffer, size_t length, uint32_t timeout, bool expect_excess_bytes, bool is_flush);

static int smc_occp_handle_get_status(interface_driver_t drv, driver_type_t drv_type, occp_header hdr, bool body_crc_present);
static int smc_occp_handle_get_version(interface_driver_t drv, driver_type_t drv_type, occp_header hdr, bool body_crc_present);
static int smc_occp_handle_get_boot_version(interface_driver_t drv, driver_type_t drv_type, occp_header hdr, bool body_crc_present);
static int smc_occp_handle_read(interface_driver_t drv, driver_type_t drv_type, occp_header hdr, bool body_crc_present);
static int smc_occp_handle_write(interface_driver_t drv, driver_type_t drv_type, occp_header hdr, bool body_crc_present);
static int smc_occp_handle_jump(interface_driver_t drv, driver_type_t drv_type, occp_header hdr, bool body_crc_present);
static int smc_occp_handle_validate_boot(interface_driver_t drv, driver_type_t drv_type, occp_header hdr, bool body_crc_present);
static occp_error_code_t smc_occp_handle_error_response(interface_driver_t drv, driver_type_t drv_type, occp_header hdr, Occp_ErrMsgID msgid);
/* Port: is_write added so the read-only ROM window can refuse WriteData. */
static uint8_t smc_occp_check_addr_access_allowed(uint64_t addr, uint16_t access_size, bool is_write);
static int smc_occp_flush_interface_fifo(interface_driver_t drv, driver_type_t drv_type);

static uint8_t calculate_crc8(uint8_t *buffer, size_t length);
static void crc32_init_table(void);
static uint32_t calculate_crc32(uint8_t *buffer, size_t length);

static Occp_ErrMsgID smc_occp_validate_header(packet_header hdr);
static Occp_ErrMsgID smc_occp_validate_body(uint8_t *buffer, size_t length, bool crc_present);

static int error_response_sent = 0;

/* The ROM drives a status GPIO to the host here. The board has none. */
static void set_gpio_status(occp_error_code_t status)
{
    LOG_INF("OCCP status GPIO: %s", status == OCCP_ERROR_NONE ? "ok" : "error");
}

/*********************************************************************
 * Static Global Variables
 ********************************************************************/

static smc_active_interfaces_t g_smc_active_interfaces = {0};
static occp_link_t g_i3c_link;

/* Interface latching state - initially -1 (no interface latched) */
static int g_latched_interface_index = -1;
static bool g_interface_latching_active = false;
static uint32_t g_interface_error_count = 0;
static int g_last_unlatched_interface = -1; /* Track last unlatched interface for round-robin */

/* Interface unlatch thresholds and configuration */
#define INTERFACE_ERROR_THRESHOLD 5         /* Unlatch after 5 consecutive errors */
#define INTERFACE_TIMEOUT_THRESHOLD 1000000 /* Timeout iterations before unlatch */

/* Max message body plus up to 4 bytes CRC overhead */
static uint8_t g_occp_data_buffer[OCCP_MAX_MSG_SIZE + 4];
static uint8_t read_response_packet_buffer[sizeof(packet_header) + OCCP_MAX_RD_SIZE + sizeof(uint32_t)];

/* For the shell. */
occp_link_t *occp_target_link(void)
{
    return g_smc_active_interfaces.num_channels > 0 ? g_smc_active_interfaces.channel_drivers[0] : NULL;
}

/**
 * @brief Calculates CRC8 checksum for a given data buffer using polynomial 0xD3.
 */
static uint8_t calculate_crc8(uint8_t *data, size_t length)
{
    // x^8 +x^7 +x^5 +x^2 +x +1
    uint8_t crc = 0xFF; // Initial value

    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

// Precomputed CRC32 table for polynomial 0x992C14AC
static uint32_t crc32_table[256];
static bool crc32_table_initialized = false;

// Generate the CRC32 table at runtime (called once)
static void crc32_init_table(void)
{
    for (uint32_t i = 0; i < 256; i++)
    {
        uint32_t crc = i << 24;
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x80000000)
                crc = (crc << 1) ^ CRC32_POLYNOMIAL;
            else
                crc <<= 1;
        }
        crc32_table[i] = crc;
    }
    crc32_table_initialized = true;
}

static uint32_t calculate_crc32(uint8_t *data, size_t length)
{
    if (!crc32_table_initialized)
        crc32_init_table();

    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; i++)
    {
        uint8_t idx = (uint8_t)((crc >> 24) ^ data[i]);
        crc = (crc << 8) ^ crc32_table[idx];
    }
    return crc ^ 0xFFFFFFFF;
}

static Occp_ErrMsgID smc_occp_validate_header(packet_header hdr)
{
    // Validate the header fields
    // only calculate crc on the 56 bits of header fields

    uint8_t *hdr_ptr = ((uint8_t *)&hdr) + 1;

    uint8_t calculated_crc = calculate_crc8(hdr_ptr, 7);
    if (hdr.hdr_crc != calculated_crc)
    {
        simputshex16("OCCP: Header CRC mismatch. Calculated: ", calculated_crc);
        simputshex16("OCCP: Header CRC mismatch. Received: ", hdr.hdr_crc);
        return Corrupt_header;
    }
    return NoErr;
}
static Occp_ErrMsgID smc_occp_validate_body(uint8_t *data_buffer, size_t data_len, bool crc_present)
{
    simputshex16("OCCP: Validating body of length: ", data_len);
    if (data_len == 0) // This condition should not be hit, as the calling function should ahve checked this earlier
    {
        return Invalid_header;
    }
    if (crc_present)
    {
        if (data_len > (PACKET_SIZE_FOR_CRC8 + sizeof(uint32_t))) // If body length > 14 bytes, then data_len = body_len + 4 byte CRC will be >18; use CRC32
        {
            uint32_t crc32_val;
            memcpy(&crc32_val, data_buffer + data_len - 4, sizeof(uint32_t)); // data_len = message_length+4 byte crc32; last 4 bytes are crc32
            simputshex32("OCCP: Validating body CRC32, received CRC32: ", crc32_val);
            if (calculate_crc32(data_buffer, (data_len - 4)) == crc32_val)
            {
                return NoErr;
            }
            else
            {
                return Corrupt_Data;
            }
        }
        else
        {
            simputshex16("OCCP: Validating body CRC8, received CRC8: ", data_buffer[data_len - 1]);
            if (calculate_crc8(data_buffer, data_len - 1) == data_buffer[data_len - 1]) // data_len = message_length+1 byte crc8; last byte is crc8
            {
                return NoErr;
            }
            else
            {
                simputs("OCCP: Body CRC8 mismatch, returning Corrupt_Data\n");
                return Corrupt_Data;
            }
        }
    }
    else
    {
        return NoErr;
    }
}

int smc_occp_init(void)
{
    occp_error_code_t ret = OCCP_ERROR_NONE;
    uint64_t i3c_id = 0x0;

    smc_occp_init_transport_timeout();
    occp_mem_init();

    /*
     * The ROM opens I3C channels 0, 1 and 3 and I2C channels 0 and 1, each
     * with an ID from eFuse slots. This board has one I3C target instance.
     * The I2C channel is added in Phase 6 of the bench plan.
     */
    i3c_id = smc_occp_determine_i3c_address(0x0);
    ret |= smc_occp_init_i3c_channel(true, 0, i3c_id);

    if (ret != OCCP_ERROR_NONE)
    {
        LOG_ERR("OCCP interface initialization failed");
        occp_status_set_interface_status(OCCP_INTERFACE_STATUS_ERROR);
        set_gpio_status(OCCP_ERROR_INTERFACE_ERROR);
        return ret;
    }
    else
    {
        LOG_INF("OCCP interface initialized: %u channel(s)", (unsigned int)g_smc_active_interfaces.num_channels);
        occp_status_set_interface_status(OCCP_INTERFACE_STATUS_READY);
        set_gpio_status(OCCP_ERROR_NONE);
        return ret;
    }
}

void smc_occp_force_unlatch(void)
{
    simputs("OCCP: Force unlatch requested\n");
    smc_occp_unlatch_interface();
}

static int smc_occp_poll_channels(void)
{
    static uint32_t timeout_counter = 0;

    while (1)
    {
        if (g_interface_latching_active && g_latched_interface_index >= 0)
        {
            /* Interface already latched - only check the latched interface */
            if (smc_occp_interface_has_data(g_latched_interface_index))
            {
                timeout_counter = 0; /* Reset timeout on successful data */
                return g_latched_interface_index;
            }

            /* Check for timeout on latched interface */
            timeout_counter++;
            if (timeout_counter >= INTERFACE_TIMEOUT_THRESHOLD)
            {
                simputs("OCCP: Latched interface timeout, attempting unlatch\n");
                smc_occp_unlatch_interface();
                timeout_counter = 0; /* Reset counter */
            }
        }
        else
        {
            /* No interface latched yet - poll all enabled interfaces */
            /* Implement round-robin: start polling from interface after last unlatched */
            int start_index = (g_last_unlatched_interface >= 0) ? (g_last_unlatched_interface + 1) % (int)g_smc_active_interfaces.num_channels : 0;

            int i;
            for (i = 0; i < (int)g_smc_active_interfaces.num_channels; i++)
            {
                int current_index = (start_index + i) % (int)g_smc_active_interfaces.num_channels;
                if (smc_occp_interface_has_data(current_index))
                {
                    timeout_counter = 0; /* Reset timeout when data found */
                    return current_index;
                }
            }
        }

        /*
         * Port: the ROM busy-polls the hardware here. On Zephyr that would
         * starve the shell and the log thread, so sleep until a bus
         * callback signals a complete transaction, or a short timeout.
         */
        occp_port_wait_for_data(OCCP_POLL_IDLE_WAIT);
    }
}

static void smc_occp_latch_interface(int interface_index)
{
    if (!g_interface_latching_active)
    {
        g_latched_interface_index = interface_index;
        g_interface_latching_active = true;
        g_interface_error_count = 0; /* Reset error count on successful latch */

        // Report the active interface in POST code
        switch (interface_index)
        {
        case 0:
            smc_post_code_set_interface(POST_CODE_IFACE_I3C0);
            break;
        case 1:
            smc_post_code_set_interface(POST_CODE_IFACE_I3C1);
            break;
        case 2:
            smc_post_code_set_interface(POST_CODE_IFACE_I3C3);
            break;
        case 3:
            smc_post_code_set_interface(POST_CODE_IFACE_I2C0);
            break;
        case 4:
            smc_post_code_set_interface(POST_CODE_IFACE_I2C1);
            break;
        default:
            smc_post_code_set_interface(POST_CODE_IFACE_NONE);
            break;
        }

        LOG_DBG("OCCP: interface latched to index %d", interface_index);
    }
}

static void smc_occp_unlatch_interface(void)
{
    if (g_interface_latching_active)
    {
        LOG_INF("OCCP: unlatching interface index %d", g_latched_interface_index);

        /* Option 4: Drain FIFO to prevent immediate re-latching */
        interface_driver_t drv = g_smc_active_interfaces.channel_drivers[g_latched_interface_index];
        driver_type_t drv_type = g_smc_active_interfaces.type[g_latched_interface_index];
        smc_occp_flush_interface_fifo(drv, drv_type);

        /* Option 2: Track last unlatched interface for round-robin polling */
        g_last_unlatched_interface = g_latched_interface_index;

        g_latched_interface_index = -1;
        g_interface_latching_active = false;
        g_interface_error_count = 0;

        /* Report interface unlatch event */
        smc_status_report(SMC_STATUS_TYPE_WARNING, SMC_OCCP_ERROR_WITH_DATA(SMC_OCCP_ERROR_CMD_FAILED, OCCP_ERROR_INTERFACE_ERROR));
    }
}

static bool smc_occp_handle_interface_error(void)
{
    if (g_interface_latching_active)
    {
        g_interface_error_count++;
        simputshex32("OCCP: Interface error count: ", g_interface_error_count);

        if (g_interface_error_count >= INTERFACE_ERROR_THRESHOLD)
        {
            simputs("OCCP: Interface error threshold exceeded, unlatching\n");
            smc_occp_unlatch_interface();
            return true; /* Interface was unlatched */
        }
    }
    return false; /* Interface still latched */
}

static bool smc_occp_interface_has_data(int interface_index)
{
    if (interface_index < 0 || (size_t)interface_index >= g_smc_active_interfaces.num_channels)
    {
        return false;
    }

    interface_driver_t drv = g_smc_active_interfaces.channel_drivers[interface_index];
    if (drv == NULL)
    {
        return false;
    }

    /* Port: both bus types present the same mailbox. */
    return occp_link_has_data(drv);
}
static void smc_occp_handle_transport_error(interface_driver_t drv, driver_type_t drv_type, packet_header command_packet, bool hdr_valid, occp_error_code_t occp_status)
{
    if (hdr_valid == 0)
    {
        command_packet.hdr.app_id = 0xFF;
        command_packet.hdr.msg_id = 0xFF;
        command_packet.hdr.flags = 0x0;
    }
    if (occp_status == OCCP_ERROR_TRANSPORT_INCOMPLETE)
    {
        smc_occp_handle_error_response(drv, drv_type, command_packet.hdr, Incomplete_msg); // Incomplete transaction
    }
    else if (occp_status == OCCP_ERROR_TRANSPORT_OVERFLOW)
    {
        smc_occp_handle_error_response(drv, drv_type, command_packet.hdr, Oversize_msg); // Overflow transaction
    }
    else
    {
        smc_occp_handle_error_response(drv, drv_type, command_packet.hdr, Transport_crc); // Generic transport CRC, SMC Boot ROM will not generate this error
    }
}

void smc_occp_process(void)
{
    packet_header command_packet;
    occp_header command_word;
    // Set initial OCCP state to IDLE
    smc_post_code_set_occp_state(POST_CODE_OCCP_STATE_IDLE);

    while (1)
    {
        /* Reset error_response_sent flag at the start of each command iteration */
        error_response_sent = 0;

        int channel_with_data = smc_occp_poll_channels();
        interface_driver_t drv = g_smc_active_interfaces.channel_drivers[channel_with_data];
        driver_type_t drv_type = g_smc_active_interfaces.type[channel_with_data];
        occp_error_code_t occp_status = OCCP_ERROR_INVALID_COMMAND; // initialize to invalid command
        occp_error_code_t ret = OCCP_ERROR_TIMEOUT;                 // initialize to timeout error
        if (!drv)
        {
            simputs("[OCCP] Channel driver is NULL\n");
            continue; // Skip to next iteration if no valid driver
        }
        occp_status = smc_occp_read_from_bus_4byte_aligned_or_complete_stream(drv, drv_type, (uint8_t *)&command_packet, sizeof(command_packet), TRANSPORT_TIMEOUT, 1, 0);
        // Set state to indicate command being received
        smc_post_code_set_occp_state(POST_CODE_OCCP_STATE_CMD_RECEIVED);

        if (occp_status == OCCP_ERROR_NONE)
        {
            // Set state to indicate command processing
            smc_post_code_set_occp_state(POST_CODE_OCCP_STATE_PROCESSING);

            /*
            Received packet header format
            <8-bit Header CRC | 24-bit rsvd |32-bit OCCP Header>
            Packet structure is
            <64bit header| Message Body(1024max)|Body CRC (8 /32 bit)>
            */
            // Check for Header CRC pass
            Occp_ErrMsgID header_validation = smc_occp_validate_header(command_packet);

            if (header_validation != NoErr)
            {
                LOG_WRN("OCCP: invalid header (crc)");
                // Set error state for header validation failure
                smc_post_code_set_occp_state(POST_CODE_OCCP_STATE_ERROR);
                smc_post_code_set_error(POST_CODE_ERROR_COMMAND);
                smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_WITH_DATA(SMC_OCCP_ERROR_CMD_FAILED, header_validation)); /* Command failed */
                ret = smc_occp_handle_error_response(drv, drv_type, command_packet.hdr, header_validation);                       // Header error
            }
            else
            {
                command_word = command_packet.hdr;
                LOG_DBG("OCCP: app %u msg %u len %u crc %u", command_word.app_id, command_word.msg_id,
                        command_word.length, command_packet.body_crc_present);
                if (command_word.app_id == Base)
                {
                    if (command_word.msg_id < Base_max_command)
                    {
                        // Found proper Base command, latch to this interface
                        smc_occp_latch_interface(channel_with_data);
                    }

                    switch (command_word.msg_id)
                    {
                    case GetVersion_base:
                        ret = smc_occp_handle_get_version(drv, drv_type, command_word, command_packet.body_crc_present);
                        break;
                    case GetStatus:
                        ret = smc_occp_handle_get_status(drv, drv_type, command_word, command_packet.body_crc_present);
                        break;
                    case WriteData:
                        ret = smc_occp_handle_write(drv, drv_type, command_word, command_packet.body_crc_present);
                        break;
                    case ReadData:
                        ret = smc_occp_handle_read(drv, drv_type, command_word, command_packet.body_crc_present);
                        break;
                    default:
                        simputshex32("Unknown OCCP command received: ", command_word.msg_id);
                        smc_post_code_set_occp_state(POST_CODE_OCCP_STATE_ERROR);
                        smc_post_code_set_error(POST_CODE_ERROR_COMMAND);
                        smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_WITH_DATA(SMC_OCCP_ERROR_CMD_UNKNOWN, command_word.msg_id)); /* Unknown command */
                        ret = smc_occp_handle_error_response(drv, drv_type, command_word, Invalid_Msgid);                                    // MsgID error
                        break;
                    }
                }
                else if (command_word.app_id == Boot)
                {
                    if (command_word.msg_id < Boot_max_command)
                    { // Proper Boot command, latch to this interface
                        smc_occp_latch_interface(channel_with_data);
                    }
                    switch (command_word.msg_id)
                    {
                    case GetVersion_boot:
                        ret = smc_occp_handle_get_boot_version(drv, drv_type, command_word, command_packet.body_crc_present);
                        break;
                    case ExecuteImage:
                        ret = smc_occp_handle_jump(drv, drv_type, command_word, command_packet.body_crc_present);
                        break;
                    case AuthenticateImage:
                        ret = smc_occp_handle_validate_boot(drv, drv_type, command_word, command_packet.body_crc_present);
                        break;
                    case Train_d2d:
                        simputs("OCCP: Train_d2d command not implemented in Boot ROM\n");
                        smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_WITH_DATA(SMC_OCCP_ERROR_CMD_UNKNOWN, command_word.msg_id)); /* Unknown command */
                        ret = smc_occp_handle_error_response(drv, drv_type, command_word, Unsupported_StatusID);                             // MsgID error
                        break;
                    default:
                        simputshex32("Unknown OCCP command received: ", command_word.msg_id);
                        smc_post_code_set_occp_state(POST_CODE_OCCP_STATE_ERROR);
                        smc_post_code_set_error(POST_CODE_ERROR_COMMAND);
                        smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_WITH_DATA(SMC_OCCP_ERROR_CMD_UNKNOWN, command_word.msg_id)); /* Unknown command */
                        ret = smc_occp_handle_error_response(drv, drv_type, command_word, Invalid_Msgid);                                    // MsgID error
                        break;
                    }
                }
                else
                {

                    simputshex16("OCCP: Invalid Application received: ", command_word.app_id);
                    smc_post_code_set_occp_state(POST_CODE_OCCP_STATE_ERROR);
                    smc_post_code_set_error(POST_CODE_ERROR_COMMAND);
                    smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_WITH_DATA(SMC_OCCP_ERROR_CMD_UNKNOWN, command_word.app_id)); /* Unknown command */
                    ret = smc_occp_handle_error_response(drv, drv_type, command_word, Invalid_Appid);
                }
            }
        }
        else
        {
            // Set error state for command read failure
            smc_post_code_set_occp_state(POST_CODE_OCCP_STATE_ERROR);
            smc_post_code_set_error(POST_CODE_ERROR_COMMAND);

            if (occp_status == OCCP_ERROR_TRANSPORT_INCOMPLETE || occp_status == OCCP_ERROR_TRANSPORT_OVERFLOW)
            {
                // If the first command read is incomplete, flush the fifo to clear any partial data and wait for a new command
                bool hdr_valid = 0;
                // Handle transport error for incomplete/overflow reads
                smc_post_code_set_occp_state(POST_CODE_OCCP_STATE_ERROR);
                smc_post_code_set_error(POST_CODE_ERROR_INTERFACE);
                smc_occp_handle_transport_error(drv, drv_type, command_packet, hdr_valid, occp_status);
            }
            LOG_WRN("OCCP: error reading command from bus (%d)", occp_status);
            occp_status_set_error_code(ret);
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_CMD_READ); /* Command read error */
            /* Handle interface error and potentially unlatch */
            if (smc_occp_handle_interface_error())
            {
                simputs("OCCP: Interface unlatched due to errors, retrying with all interfaces\n");
            }
            continue; /* Retry reading the command. */
        }
        if (ret == OCCP_ERROR_NONE)
        {
            /* Reset error count on successful command */
            smc_post_code_set_occp_state(POST_CODE_OCCP_STATE_COMPLETE);
            smc_post_code_set_error(POST_CODE_ERROR_NONE);
            g_interface_error_count = 0;
        }
        else
        {
            /* Set error state for command execution failure */
            smc_post_code_set_occp_state(POST_CODE_OCCP_STATE_ERROR);
            smc_post_code_set_error(POST_CODE_ERROR_COMMAND);
            occp_status_set_error_code(ret);
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_WITH_DATA(SMC_OCCP_ERROR_CMD_FAILED, ret)); /* Command failed */
            if (!error_response_sent)
            {
                // avoid flushing the fifo twice (already done if error response was sent)
                smc_occp_flush_interface_fifo(drv, drv_type);
            }

            /* Handle interface error for command execution failures */
            smc_occp_handle_interface_error();
        }
        occp_status_increment_command_count();
    }
}

static occp_error_code_t smc_occp_handle_error_response(interface_driver_t drv, driver_type_t drv_type, occp_header hdr, Occp_ErrMsgID err)
{
    LOG_WRN("OCCP: error response 0x%x", (unsigned int)err);
    /* Fill the occp_error_resp struct fields correctly. */
    error_response err_response;
    // Access header fields through the hdr member
    if (err == Corrupt_header)
    {
        err_response.hdr.app_id = 0xFF;
        err_response.hdr.msg_id = 0xFF;
    }
    else
    {
        err_response.hdr.app_id = hdr.app_id;
        err_response.hdr.msg_id = hdr.msg_id;
    }
    err_response.hdr.flags = hdr.flags;
    err_response.hdr.error = 1;                 // Indicate error status
    err_response.hdr.length = sizeof(uint32_t); // Length of error_code field
    err_response.body_crc_present = true;       // Error response includes body CRC
    err_response.i3c_flags = 0;                 // Reserved
    err_response.err_code = (uint32_t)err;
    uint8_t *hdr_ptr = ((uint8_t *)&err_response) + 1;
    err_response.hdr_crc = calculate_crc8(hdr_ptr, 7); // CRC over 7 bytes after hdr_crc
    // Calculate body CRC
    uint8_t *body_ptr = (uint8_t *)&err_response + 8; // Point to the start of the body (after header)
    size_t body_len = sizeof(uint32_t);               // Length of the body (error_code field)
    err_response.body_crc = calculate_crc8(body_ptr, body_len);

    /*IMPORTANT: When header validation fails (corrupted CRC, invalid app/msg IDs),we must flush the interface FIFO to prevent body data from the failed command
    from being interpreted as the next command header. This prevents the ROM from entering an unpredictable state due to header corruption.*/
    smc_occp_flush_interface_fifo(drv, drv_type);

    // Set response ready state before sending error response
    smc_post_code_set_occp_state(POST_CODE_OCCP_STATE_RESP_READY);

    /* Send error response to host after clearing the rx fifo */
    smc_occp_send_to_bus(drv, drv_type, (uint8_t *)&err_response, sizeof(err_response), TRANSPORT_TIMEOUT);

    /* Mark that error response was sent and FIFO was already flushed */
    error_response_sent = 1;

    return OCCP_ERROR_INVALID_COMMAND;
}

static int smc_occp_handle_get_boot_version(interface_driver_t drv, driver_type_t drv_type, occp_header hdr, bool body_crc_present)
{
    get_version_response response;
    simputs("Handling GET_BOOT_VERSION command\n");

    // Check for length of the command
    uint16_t count_len_bytes = hdr.length; // Length of the body as per header
    if (count_len_bytes != GET_VER_HEADER_LENGTH)
    {
        simputshex16("Error in Get Version Command length, Ignoring GET_VERSION Command: ", count_len_bytes);
        smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_header);
        smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_CMD_FAILED);
        return OCCP_ERROR_INVALID_COMMAND;
    }

    // check to ensure no extra bytes are present in the FIFO
    int flushed_bytes = smc_occp_flush_interface_fifo(drv, drv_type);
    if (flushed_bytes != 0)
    {
        simputs("Oversize message for GET_VERSION command\n");
        smc_occp_handle_error_response(drv, drv_type, hdr, Oversize_msg);
        return OCCP_ERROR_TRANSPORT_OVERFLOW;
    }

    // Creating get version response header
    response.body_crc_present = true; // Body CRC for version response
    response.i3c_flags = 0;           // Reserved
    response.hdr.error = 0;           // No error
    response.hdr.flags = hdr.flags;   // Echo back flags from request
    response.hdr.app_id = hdr.app_id; // Echo back app_id from request
    response.hdr.msg_id = hdr.msg_id; // Echo back msg_id from request
    response.hdr.length = 4;          // Length of status field (4 bytes)
    // Calculate header CRC
    uint8_t *hdr_ptr = ((uint8_t *)&response) + 1;
    response.hdr_crc = calculate_crc8(hdr_ptr, 7); // CRC over 7 bytes after hdr_crc
    // Creating get version response
    response.major_version = BOOT_VERSION_MAJOR;
    response.minor_version = BOOT_VERSION_MINOR;
    response.patch_version = BOOT_VERSION_PATCH;
    // Calculate body CRC over exactly 'length' bytes of body (exclude CRC byte)
    uint8_t *body_ptr = (uint8_t *)&response + 8; // Start of body (after 8-byte packet header)
    size_t body_len = response.hdr.length;        // 4-byte version body
    response.body_crc = calculate_crc8(body_ptr, body_len);

    // Set response ready state before sending
    smc_post_code_set_occp_state(POST_CODE_OCCP_STATE_RESP_READY);

    return smc_occp_send_to_bus(drv, drv_type, (uint8_t *)&response, sizeof(response), TRANSPORT_TIMEOUT);
}
static int smc_occp_handle_get_version(interface_driver_t drv, driver_type_t drv_type, occp_header hdr, bool body_crc_present)
{
    get_version_response response;
    simputs("Handling GET_VERSION command\n");
    // check for length of the command
    uint16_t count_len_bytes = hdr.length; // Length of the body as per header
    if (count_len_bytes != GET_VER_HEADER_LENGTH)
    {
        simputshex16("Error in Get Version Command length, Ignoring GET_VERSION Command: ", count_len_bytes);
        smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_header);
        smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_CMD_FAILED);
        return OCCP_ERROR_INVALID_COMMAND;
    }

    // check to ensure no extra bytes are present in the FIFO
    int flushed_bytes = smc_occp_flush_interface_fifo(drv, drv_type);
    if (flushed_bytes != 0)
    {
        simputs("Oversize message for GET_VERSION command\n");
        smc_occp_handle_error_response(drv, drv_type, hdr, Oversize_msg);
        return OCCP_ERROR_TRANSPORT_OVERFLOW;
    }

    // Creating get version response header
    response.body_crc_present = true; // Body CRC for version response
    response.i3c_flags = 0;           // Reserved
    response.hdr.error = 0;           // No error
    response.hdr.flags = hdr.flags;   // Echo back flags from request
    response.hdr.app_id = hdr.app_id; // Echo back app_id from request, will be 0x0
    response.hdr.msg_id = hdr.msg_id; // Echo back msg_id from request, will be 0x0
    response.hdr.length = 4;          // Length of version fields (4 bytes)
    // Calculate header CRC
    uint8_t *hdr_ptr = ((uint8_t *)&response) + 1;
    response.hdr_crc = calculate_crc8(hdr_ptr, 7); // CRC over 7 bytes after hdr_crc
    // Creating get version response
    response.major_version = OCCP_VERSION_MAJOR;
    response.minor_version = OCCP_VERSION_MINOR;
    response.patch_version = OCCP_VERSION_PATCH;
    // Calculate body CRC over exactly 'length' bytes of body (exclude CRC byte)
    uint8_t *body_ptr = (uint8_t *)&response + 8; // Start of body (after 8-byte packet header)
    size_t body_len = response.hdr.length;        // 4-byte version body
    response.body_crc = calculate_crc8(body_ptr, body_len);

    // Set response ready state before sending
    smc_post_code_set_occp_state(POST_CODE_OCCP_STATE_RESP_READY);

    return smc_occp_send_to_bus(drv, drv_type, (uint8_t *)&response, sizeof(response), TRANSPORT_TIMEOUT);
}

static int smc_occp_handle_get_status(interface_driver_t drv, driver_type_t drv_type, occp_header hdr, bool body_crc_present)
{
    if (!smc_strap_is_status_rpt_disable())
    {
        uint32_t occp_status_reg = 0;
        uint32_t status = 0;
        uint8_t status_buf[GET_STAT_HEADER_LENGTH + 1] = {0};
        uint16_t status_id = 0;
        Occp_ErrMsgID body_validation = Corrupt_Data;
        occp_error_code_t command_status = OCCP_ERROR_GENERAL;
        get_status_response response;

        uint16_t count_len_bytes = hdr.length;

        // Check for correct command length
        if (count_len_bytes != GET_STAT_HEADER_LENGTH)
        {
            simputshex16("Error in Status ID length, Ignoring GET_STATUS Command: ", count_len_bytes);
            smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_header);
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_CMD_FAILED);
            return OCCP_ERROR_INVALID_COMMAND;
        }

        // Add CRC length if present
        if (body_crc_present)
        {
            if (count_len_bytes > PACKET_SIZE_FOR_CRC8)
                count_len_bytes += sizeof(uint32_t); // Add crc32 length
            else
                count_len_bytes += sizeof(uint8_t); // Add crc8 length
        }

        // Read the command
        command_status = smc_occp_read_from_bus_4byte_aligned_or_complete_stream(drv, drv_type, (uint8_t *)&status_buf, count_len_bytes, TRANSPORT_TIMEOUT, 0, 0);

        if (command_status == OCCP_ERROR_NONE)
        {
            body_validation = smc_occp_validate_body((uint8_t *)&status_buf, count_len_bytes, body_crc_present);
            if (body_validation == NoErr)
            {
                memcpy(&status_id, &status_buf, hdr.length);
                simputshex16("GET_STATUS command with Status ID: ", status_id);
                if (status_id <= 0xFF)
                {
                    occp_status_reg = occp_status_get();
                    switch (status_id)
                    {
                    case 0:
                        status = ((uint32_t)(occp_status_reg & 0xF)) | ((uint32_t)SMC_STATUS_FW_ID_SMC_BL0 << 16) | ((uint32_t)SMC_STATUS_TYPE_STATUS << 24);
                        simputshex32("Boot status: ", status);
                        break;
                    case 1:
                        status = ((uint32_t)((occp_status_reg >> 4) & 0xF)) | ((uint32_t)SMC_STATUS_FW_ID_SMC_BL0 << 16) | ((uint32_t)SMC_STATUS_TYPE_STATUS << 24);
                        break;
                    case 2:
                        status = (uint32_t)((occp_status_reg >> 8) & 0xFFFF);
                        status |= ((uint32_t)SMC_STATUS_FW_ID_SMC_BL0 << 16);
                        status |= ((uint32_t)SMC_STATUS_TYPE_STATUS << 24);
                        break;
                    case 3:
                        status = (uint32_t)((occp_status_reg >> 16) & 0xFF);
                        status |= ((uint32_t)SMC_STATUS_FW_ID_SMC_BL0 << 16);
                        status |= ((uint32_t)SMC_STATUS_TYPE_ERROR << 24);
                        break;
                    default:
                        simputshex16("Error in Status ID, Ignoring GET_STATUS Command: ", status_id);
                        smc_occp_handle_error_response(drv, drv_type, hdr, Unsupported_StatusID);
                        smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_CMD_FAILED);
                        return OCCP_ERROR_INVALID_COMMAND;
                    }
                }
                else if (status_id == 0x8000)
                {
                    sep_status_read(&status);
                    simputshex32("SEP status read: ", status);
                }
                else if (status_id == 0x8001)
                {
                    smc_status_read(&status);
                    simputshex32("SMC status read: ", status);
                }
                else
                {
                    simputshex16("Error in Status ID, Ignoring GET_STATUS Command: ", status_id);
                    smc_occp_handle_error_response(drv, drv_type, hdr, Unsupported_StatusID);
                    smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_CMD_FAILED);
                    return OCCP_ERROR_INVALID_COMMAND;
                }
            }
            else
            {
                simputs("Error in Status ID, Ignoring GET_STATUS Command");
                smc_occp_handle_error_response(drv, drv_type, hdr, body_validation);
                smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_CMD_FAILED);
                return OCCP_ERROR_CRC;
            }

            response.body_crc_present = true;
            response.i3c_flags = 0;
            response.hdr.error = 0;
            response.hdr.flags = hdr.flags;
            response.hdr.app_id = hdr.app_id;
            response.hdr.msg_id = hdr.msg_id;
            response.hdr.length = 4;
            uint8_t *hdr_ptr = ((uint8_t *)&response) + 1;
            response.hdr_crc = calculate_crc8(hdr_ptr, 7);
            response.status = status;
            simputshex32("GET_STATUS response with Status: ", response.status);
            uint8_t *body_ptr = (uint8_t *)&response + 8;
            size_t body_len = response.hdr.length;
            response.body_crc = calculate_crc8(body_ptr, body_len);
            // Set response ready state before sending
            smc_post_code_set_occp_state(POST_CODE_OCCP_STATE_RESP_READY);

            return smc_occp_send_to_bus(drv, drv_type, (uint8_t *)&response, sizeof(response), TRANSPORT_TIMEOUT);
        }
        else if (command_status == OCCP_ERROR_TRANSPORT_INCOMPLETE)
        {
            simputs("Error reading GET_STATUS command from bus (incomplete)\n");
            smc_occp_handle_error_response(drv, drv_type, hdr, Incomplete_msg);
            return OCCP_ERROR_TRANSPORT_INCOMPLETE;
        }
        else if (command_status == OCCP_ERROR_TRANSPORT_OVERFLOW)
        {
            simputs("Error reading GET_STATUS command from bus (oversize)\n");
            smc_occp_handle_error_response(drv, drv_type, hdr, Oversize_msg);
            return OCCP_ERROR_TRANSPORT_OVERFLOW;
        }
    }
    else
    {
        simputs("Status reporting is disabled. Ignoring GET_STATUS command\n");
        smc_occp_handle_error_response(drv, drv_type, hdr, Unsupported_StatusID);
        return OCCP_ERROR_INVALID_COMMAND;
    }
    return OCCP_ERROR_INVALID_COMMAND;
}

static int smc_occp_handle_write(interface_driver_t drv, driver_type_t drv_type, occp_header hdr, bool body_crc_present)
{
    simputs("Handling WRITE command\n");
    uint16_t count_len_bytes = hdr.length;
    uint64_t addr;
    packet_header write_response_header;
    Occp_ErrMsgID body_validation = Corrupt_Data;

    simputshex16("WRITE command length in bytes: ", count_len_bytes);
    /* Check for zero-length transfer */
    if (count_len_bytes == 0)
    {
        simputs("WRITE command with zero length not allowed\n");
        smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_WRITE_OVERFLOW);
        smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_header);
        return OCCP_ERROR_BUFFER_OVERFLOW;
    }
    if (count_len_bytes < WRITE_HEADER_LENGTH)
    {
        simputshex16("WRITE command length shorter than header: ", count_len_bytes);
        smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_WRITE_OVERFLOW);
        smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_header);
        return OCCP_ERROR_BUFFER_OVERFLOW;
    }
    /* Check for Max length overflow */
    if (count_len_bytes > OCCP_MAX_MSG_SIZE)
    {
        simputshex32("WRITE count would cause overflow: ", count_len_bytes);
        simputshex16("Maximum allowed count: ", OCCP_MAX_MSG_SIZE);
        smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_WRITE_OVERFLOW);
        smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_header);
        return OCCP_ERROR_BUFFER_OVERFLOW;
    }

    if (body_crc_present)
    {
        if (count_len_bytes > PACKET_SIZE_FOR_CRC8)
        {
            count_len_bytes = count_len_bytes + sizeof(uint32_t); // Account for 4-byte CRC
        }
        else
        {
            count_len_bytes = count_len_bytes + sizeof(uint8_t); // Account for 1-byte CRC
        }
    }
    /* WRITE command format per spec: 16 bit reserved + 5 bit attr +11 bit len + 8-byte address + data +crc*/
    occp_error_code_t ret = smc_occp_read_from_bus_4byte_aligned_or_complete_stream(drv, drv_type, g_occp_data_buffer, count_len_bytes, TRANSPORT_TIMEOUT, 0, 0);
    if (ret != OCCP_ERROR_NONE)
    {
        simputs("Error reading WRITE command header from bus\n");
        if (ret == OCCP_ERROR_TRANSPORT_INCOMPLETE)
        {
            smc_occp_handle_error_response(drv, drv_type, hdr, Incomplete_msg); // Incomplete transaction
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_CMD_FAILED);
            return OCCP_ERROR_TRANSPORT_INCOMPLETE;
        }
        else if (ret == OCCP_ERROR_TRANSPORT_OVERFLOW)
        {
            smc_occp_handle_error_response(drv, drv_type, hdr, Oversize_msg); // Oversize transaction
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_CMD_FAILED);
            return OCCP_ERROR_TRANSPORT_OVERFLOW;
        }
        return ret;
    }
    simputshex16("Body Crc present flag: ", body_crc_present);
    body_validation = smc_occp_validate_body(g_occp_data_buffer, count_len_bytes, body_crc_present);
    if (body_validation != NoErr)
    {
        simputs("Error in WRITE command body, Ignoring WRITE Command \n");
        smc_occp_handle_error_response(drv, drv_type, hdr, body_validation); // Body error
        smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_CMD_FAILED);
        return OCCP_ERROR_INVALID_COMMAND;
    }
    // OCCP WRITE command format:
    // - Addr (bits 95-32): 64-bit write address.
    // - WLen (bits 106-96): 11-bit write length in bytes.
    // - AddrAttr (bits 111-107): 5-bit address attributes.
    // - Reserved (bits 127-112): 16 bits reserved.
    memcpy(&addr, g_occp_data_buffer, sizeof(addr));
    uint16_t write_size;
    memcpy(&write_size, &g_occp_data_buffer[8], sizeof(write_size));
    write_size &= 0x7FF; // 11-bit length field

    if (write_size == 0)
    {
        simputs("WRITE command with zero length not allowed\n");
        smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_WRITE_OVERFLOW);
        smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_header); // was Invalid_req_len
        return OCCP_ERROR_BUFFER_OVERFLOW;
    }
    if (write_size > OCCP_MAX_WR_SIZE)
    {
        simputshex16("WRITE command size exceeds maximum after bounds check: ", write_size);
        smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_WRITE_OVERFLOW);
        smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_header); // was Invalid_req_len
        return OCCP_ERROR_BUFFER_OVERFLOW;
    }

    /* Check if address is 32-bit or 64-bit aligned. If not, return Invalid_Address error code. */
    if (addr % 4 != 0)
    {
        simputs("Write address is not 32-bit aligned\n");
        smc_post_code_set_error(POST_CODE_ERROR_ACCESS);
        smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_WRITE_ACCESS_DENIED);
        smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_Address);
        return OCCP_ERROR_ACCESS_VIOLATION;
    }

    LOG_DBG("OCCP: write %u bytes at 0x%016llx", write_size, (unsigned long long)addr);

    uint8_t *data_buffer_ptr = g_occp_data_buffer + 12; // The rest then is data
    ret = smc_occp_check_addr_access_allowed(addr, write_size, true);
    if (ret != OCCP_ERROR_NONE)
    {
        simputshex16("Write denied. Access check returned code: ", ret);
        smc_post_code_set_error(POST_CODE_ERROR_ACCESS);
        smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_WITH_NIBBLE(SMC_OCCP_ERROR_WRITE_ACCESS_DENIED, ret)); /* WRITE access denied */
        smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_Address);                                           // Addr is invalid
        simputshex16("Write command processed Return code: ", ret);
        return ret;
    }
    else
    {
        /*
         * Port: the ROM writes registers in full width and SRAM byte by
         * byte. The emulated window is plain memory, so a byte copy of the
         * little-endian stream gives the same result for every width.
         */
        occp_mem_write(addr, data_buffer_ptr, write_size);
    }
    simputshex16("Write command processed Return code: ", ret);
    write_response_header.body_crc_present = false;                     // No body CRC for write response
    write_response_header.i3c_flags = 0;                                // Reserved
    write_response_header.hdr.error = (ret == OCCP_ERROR_NONE) ? 0 : 1; // Indicate error status
    write_response_header.hdr.flags = hdr.flags;                        // Echo back flags from request
    write_response_header.hdr.app_id = hdr.app_id;                      // Echo back app_id from request
    write_response_header.hdr.msg_id = hdr.msg_id;                      // Echo back msg_id from request
    write_response_header.hdr.length = 0;                               // No body for write response
    // Calculate header CRC
    uint8_t *hdr_ptr = ((uint8_t *)&write_response_header) + 1;
    write_response_header.hdr_crc = calculate_crc8(hdr_ptr, 7);
    simputshex16("Write response header crc: ", write_response_header.hdr_crc);

    // Set response ready state before sending
    smc_post_code_set_occp_state(POST_CODE_OCCP_STATE_RESP_READY);
    smc_occp_send_to_bus(drv, drv_type, (uint8_t *)&write_response_header, sizeof(write_response_header), TRANSPORT_TIMEOUT);
    return ret;
}

static int smc_occp_handle_read(interface_driver_t drv, driver_type_t drv_type, occp_header hdr, bool body_crc_present)
{
    simputs("Handling READ command\n");
    uint16_t count_len_bytes = hdr.length;
    uint64_t addr;
    Occp_ErrMsgID body_validation = Corrupt_Data;
    packet_header read_response_header;

    /* Check for command length  */
    if (count_len_bytes != READ_HEADER_LENGTH)
    {
        simputshex16("Error in Read Command length, Ignoring READ Command: ", count_len_bytes);
        smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_READ_OVERFLOW);
        smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_header);
        return OCCP_ERROR_BUFFER_OVERFLOW;
    }
    if (body_crc_present)
    {
        if (count_len_bytes > PACKET_SIZE_FOR_CRC8)
        {
            count_len_bytes = count_len_bytes + sizeof(uint32_t); // Account for 4-byte CRC
        }
        else
        {
            count_len_bytes = count_len_bytes + sizeof(uint8_t); // Account for 1-byte CRC
        }
    }

    /* READ data */
    occp_error_code_t ret = smc_occp_read_from_bus_4byte_aligned_or_complete_stream(drv, drv_type, g_occp_data_buffer, count_len_bytes, TRANSPORT_TIMEOUT, 0, 0);
    if (ret != OCCP_ERROR_NONE)
    {
        simputs("Error reading READ command from bus\n");
        if (ret == OCCP_ERROR_TRANSPORT_INCOMPLETE)
        {
            smc_occp_handle_error_response(drv, drv_type, hdr, Incomplete_msg); // Incomplete transaction
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_CMD_FAILED);
            return OCCP_ERROR_TRANSPORT_INCOMPLETE;
        }
        else if (ret == OCCP_ERROR_TRANSPORT_OVERFLOW)
        {
            smc_occp_handle_error_response(drv, drv_type, hdr, Oversize_msg); // Oversize transaction
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_CMD_FAILED);
            return OCCP_ERROR_TRANSPORT_OVERFLOW;
        }
        return ret;
    }

    body_validation = smc_occp_validate_body(g_occp_data_buffer, count_len_bytes, body_crc_present);
    if (body_validation == NoErr)
    {
        /**
         * Extracts fields from the OCCP data buffer:
         * - Addr (bits 95-32): 64-bit read address
         * - RLen (bits 106-96): 11-bit read length in bytes.
         * - AddrAttr (bits 111-107): 5-bit address attributes.
         */
        memcpy(&addr, g_occp_data_buffer, sizeof(addr));
        uint16_t num_bytes_to_send;
        memcpy(&num_bytes_to_send, &g_occp_data_buffer[8], sizeof(num_bytes_to_send));
        num_bytes_to_send &= 0x7FF; // 11-bit length field

        if (num_bytes_to_send == 0)
        {
            simputs("READ command with zero length not allowed\n");
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_READ_OVERFLOW);
            smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_header); // was Invalid_req_len
            return OCCP_ERROR_BUFFER_OVERFLOW;
        }

        if (num_bytes_to_send > OCCP_MAX_RD_SIZE)
        {
            simputshex16("READ command size exceeds maximum after bounds check: ", num_bytes_to_send);
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_READ_OVERFLOW);
            smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_header); // was Invalid_req_len
            return OCCP_ERROR_BUFFER_OVERFLOW;
        }

        /* Check if address is 32-bit or 64-bit aligned. If not, return Invalid_Address error code. */
        if (addr % 4 != 0)
        {
            simputs("Read address is not 32-bit aligned\n");
            smc_post_code_set_error(POST_CODE_ERROR_ACCESS);
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_READ_ACCESS_DENIED);
            smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_Address);
            return OCCP_ERROR_ACCESS_VIOLATION;
        }

        LOG_DBG("OCCP: read %u bytes at 0x%016llx", num_bytes_to_send, (unsigned long long)addr);

        ret = smc_occp_check_addr_access_allowed(addr, num_bytes_to_send, false);
        if (ret != OCCP_ERROR_NONE)
        {
            simputshex16("Read denied. Returning 0s. Access check returned code: ", ret);
            smc_post_code_set_error(POST_CODE_ERROR_ACCESS);
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_WITH_NIBBLE(SMC_OCCP_ERROR_READ_ACCESS_DENIED, ret)); /* READ access denied */
            smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_Address);                                          // Addr is invalid
            simputshex16("Read command processed Return code: ", ret);
            return ret;
        }
        else
        {
            read_response_header.body_crc_present = true;
            read_response_header.i3c_flags = 0;           // Reserved
            read_response_header.hdr.error = 0;           // No error
            read_response_header.hdr.flags = hdr.flags;   // Echo back flags from request
            read_response_header.hdr.app_id = hdr.app_id; // Echo back app_id from request
            read_response_header.hdr.msg_id = hdr.msg_id; // Echo back msg_id from request
            // Header length should reflect only the body size (data bytes), not CRC
            read_response_header.hdr.length = num_bytes_to_send;
            // Calculate header CRC
            uint8_t *hdr_ptr = ((uint8_t *)&read_response_header) + 1;
            read_response_header.hdr_crc = calculate_crc8(hdr_ptr, 7); // CRC over 7 bytes after hdr_crc
            simputshex16("Read response header crc: ", read_response_header.hdr_crc);

            /*
             * Port: the ROM reads registers in full width and SRAM byte by
             * byte. The emulated window is plain memory, so one copy gives
             * the same little-endian stream for every width.
             */
            occp_mem_read(addr, g_occp_data_buffer, num_bytes_to_send);

            /* Calculate body CRC  */
            uint8_t *body_ptr = g_occp_data_buffer;
            size_t body_len = num_bytes_to_send;
            uint32_t body_crc32 = 0;
            uint8_t body_crc8 = 0;
            if (num_bytes_to_send > PACKET_SIZE_FOR_CRC8)
            {
                body_crc32 = calculate_crc32(body_ptr, body_len);
                simputshex32("Calculated 32-bit body CRC for read data: ", body_crc32);
                memcpy(g_occp_data_buffer + num_bytes_to_send, &body_crc32, sizeof(body_crc32));
                num_bytes_to_send += sizeof(body_crc32); // Include CRC in total bytes to send
            }
            else
            {
                body_crc8 = calculate_crc8(body_ptr, body_len);
                simputshex16("Calculated 8-bit body CRC for read data: ", body_crc8);
                memcpy(g_occp_data_buffer + num_bytes_to_send, &body_crc8, sizeof(body_crc8));
                num_bytes_to_send += sizeof(body_crc8); // Include CRC in total bytes to send
            }

            // Combine header and body into one packet and send.
            // Port: the ROM used a 2 KB stack buffer here; the static one is the same size.
            uint8_t *packet_buffer = read_response_packet_buffer;
            memcpy(packet_buffer, &read_response_header, sizeof(read_response_header));                  // First part of packet is header + header crc
            memcpy(packet_buffer + sizeof(read_response_header), g_occp_data_buffer, num_bytes_to_send); // Second part is data + data crc
            // Set response ready state before sending
            smc_post_code_set_occp_state(POST_CODE_OCCP_STATE_RESP_READY);
            smc_occp_send_to_bus(drv, drv_type, packet_buffer, sizeof(read_response_header) + num_bytes_to_send, TRANSPORT_TIMEOUT); // Combined header and body send to transport layer
            return ret;
        }
    }
    else
    {
        simputs("Error in READ command body, Ignoring READ Command \n");
        smc_occp_handle_error_response(drv, drv_type, hdr, body_validation); // Body error
        smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_CMD_FAILED);
        return OCCP_ERROR_INVALID_COMMAND;
    }
    return OCCP_ERROR_CRC;
}

static int smc_occp_handle_jump(interface_driver_t drv, driver_type_t drv_type, occp_header hdr, bool body_crc_present)
{
    simputs("Handling JUMP command\n");
    uint64_t address;
    packet_header jump_response_header;
    Occp_ErrMsgID body_validation;

    uint16_t count_len_bytes = hdr.length;
    /* Check for command length  */
    if (count_len_bytes != EXEC_IMG_HEADER_LENGTH)
    {
        simputshex16("Error in Execute image Command length, Ignoring Execute image Command: ", count_len_bytes);
        smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_READ_OVERFLOW);
        smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_header);
        return OCCP_ERROR_BUFFER_OVERFLOW;
    }
    if (body_crc_present)
    {
        if (count_len_bytes > PACKET_SIZE_FOR_CRC8)
        {
            count_len_bytes += sizeof(uint32_t); // Account for 4-byte CRC
        }
        else
        {
            count_len_bytes += sizeof(uint8_t); // Account for 1-byte CRC
        }
    }

    occp_error_code_t ret = smc_occp_read_from_bus_4byte_aligned_or_complete_stream(drv, drv_type, g_occp_data_buffer, count_len_bytes, TRANSPORT_TIMEOUT, 0, 0);
    if (ret != OCCP_ERROR_NONE)
    {
        simputs("Error reading Execute command header from bus\n");
        if (ret == OCCP_ERROR_TRANSPORT_INCOMPLETE)
        {
            smc_occp_handle_error_response(drv, drv_type, hdr, Incomplete_msg); // Incomplete transaction
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_JUMP_READ_FAILED);
            return OCCP_ERROR_TRANSPORT_INCOMPLETE;
        }
        else if (ret == OCCP_ERROR_TRANSPORT_OVERFLOW)
        {
            smc_occp_handle_error_response(drv, drv_type, hdr, Oversize_msg); // Oversize transaction
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_JUMP_READ_FAILED);
            return OCCP_ERROR_TRANSPORT_OVERFLOW;
        }
        return ret;
    }
    body_validation = smc_occp_validate_body(g_occp_data_buffer, count_len_bytes, body_crc_present);
    if (body_validation != NoErr)
    {
        simputs("Error in JUMP command body, Ignoring JUMP Command");
        smc_occp_handle_error_response(drv, drv_type, hdr, body_validation);       // Body error
        smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_JUMP_READ_FAILED); /* JUMP failed - read error */
        return OCCP_ERROR_INVALID_COMMAND;
    }
    /* Extract address (8 bytes) in little-endian format as per spec */
    address = (uint64_t)g_occp_data_buffer[7] << 56 | (uint64_t)g_occp_data_buffer[6] << 48 |
              (uint64_t)g_occp_data_buffer[5] << 40 | (uint64_t)g_occp_data_buffer[4] << 32 |
              (uint64_t)g_occp_data_buffer[3] << 24 | (uint64_t)g_occp_data_buffer[2] << 16 |
              (uint64_t)g_occp_data_buffer[1] << 8 | (uint64_t)g_occp_data_buffer[0];

    if (!smc_security_is_secure_mode())
    {
        /* Validate jump address before alignment */
        if (address == 0)
        {
            simputs("Jump to NULL address not allowed\n");
            smc_post_code_set_error(POST_CODE_ERROR_ACCESS);
            smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_Address);
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_JUMP_READ_FAILED);
            return OCCP_ERROR_ACCESS_VIOLATION;
        }

        if (address % 4 != 0)
        {
            simputs("Jump address is not 32-bit aligned\n");
            smc_post_code_set_error(POST_CODE_ERROR_ACCESS);
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_JUMP_READ_FAILED);
            smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_Address);
            return OCCP_ERROR_ACCESS_VIOLATION;
        }

        simputshex64("Jump address (aligned): ", address);
        ret = smc_occp_check_addr_access_allowed(address, sizeof(uint64_t), false); /* Minimum manifest header size */
        if (ret != OCCP_ERROR_NONE)
        {
            simputs("JUMP: Jump address access denied\n");
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_JUMP_READ_FAILED);
            smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_Address);
            return OCCP_ERROR_ACCESS_VIOLATION;
        }

        /* Report jump execution */
        jump_response_header.body_crc_present = false; // No body CRC for jump response
        jump_response_header.i3c_flags = 0;            // Reserved
        jump_response_header.hdr.error = 0;            // No error
        jump_response_header.hdr.flags = hdr.flags;    // Echo back flags from request
        jump_response_header.hdr.app_id = hdr.app_id;  // Echo back app_id from request
        jump_response_header.hdr.msg_id = hdr.msg_id;  // Echo back msg_id from request
        jump_response_header.hdr.length = 0;           // No body for jump response
        // Calculate header CRC
        uint8_t *hdr_ptr = ((uint8_t *)&jump_response_header) + 1;
        jump_response_header.hdr_crc = calculate_crc8(hdr_ptr, 7);
        // Set response ready state before sending
        smc_post_code_set_occp_state(POST_CODE_OCCP_STATE_RESP_READY);

        smc_status_report(SMC_STATUS_TYPE_STATUS, SMC_OCCP_ERROR_WITH_DATA(SMC_OCCP_STATUS_JUMP_EXECUTED, (address >> 16))); /* JUMP executed */
        smc_occp_send_to_bus(drv, drv_type, (uint8_t *)&jump_response_header, sizeof(jump_response_header), TRANSPORT_TIMEOUT);
        /* Mark boot sequence complete */
        smc_post_code_set_boot_phase(POST_CODE_BOOT_PHASE_BOOT_COMPLETE);
        smc_status_report(SMC_STATUS_TYPE_STATUS, SMC_STATUS_BOOT_COMPLETE); /* Boot complete */

        /*
         * Port: the ROM jumps to the image here (goto *address). There is
         * no RISC-V image on this board, so answer like the ROM and stay in
         * the command loop.
         */
        LOG_WRN("OCCP: ExecuteImage at 0x%016llx acknowledged, not executed on this board",
                (unsigned long long)address);
    }
    else
    {
        ret = OCCP_ERROR_SECURITY_VIOLATION;
        simputs("OCCP_JUMP command received in secure mode, ignoring.\n");
        smc_post_code_set_error(POST_CODE_ERROR_INVALID_SEC_MODE);
        smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_Msgid);
        smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_JUMP_SECURITY); /* JUMP blocked - security */
    }
    return ret;
}

static int smc_occp_handle_validate_boot(interface_driver_t drv, driver_type_t drv_type, occp_header hdr, bool body_crc_present)
{
    simputs("Handling OCCP_VALIDATE_AND_BOOT command\n");
    uint64_t address;
    Occp_ErrMsgID body_validation;
    packet_header validate_response_header;

    uint16_t count_len_bytes = hdr.length;
    /* Check for command length  */
    if (count_len_bytes != AUTH_IMG_HEADER_LENGTH)
    {
        simputshex16("Error in Authenticate Image Command length, Ignoring Authenticate Image Command: ", count_len_bytes);
        smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_READ_OVERFLOW);
        smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_header);
        return OCCP_ERROR_BUFFER_OVERFLOW;
    }
    if (body_crc_present)
    {
        if (count_len_bytes > PACKET_SIZE_FOR_CRC8)
        {
            count_len_bytes += sizeof(uint32_t); // Account for 4-byte CRC
        }
        else
        {
            count_len_bytes += sizeof(uint8_t); // Account for 1-byte CRC
        }
    }

    occp_error_code_t ret = smc_occp_read_from_bus_4byte_aligned_or_complete_stream(drv, drv_type, g_occp_data_buffer, count_len_bytes, TRANSPORT_TIMEOUT, 0, 0);
    if (ret != OCCP_ERROR_NONE)
    {
        simputs("Error reading Authenticate image command header from bus\n");
        if (ret == OCCP_ERROR_TRANSPORT_INCOMPLETE)
        {
            smc_occp_handle_error_response(drv, drv_type, hdr, Incomplete_msg); // Incomplete transaction
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_VALIDATE_ADDRESS_FAILED);
            return OCCP_ERROR_TRANSPORT_INCOMPLETE;
        }
        else if (ret == OCCP_ERROR_TRANSPORT_OVERFLOW)
        {
            smc_occp_handle_error_response(drv, drv_type, hdr, Oversize_msg); // Oversize transaction
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_VALIDATE_ADDRESS_FAILED);
            return OCCP_ERROR_TRANSPORT_OVERFLOW;
        }
        return ret;
    }
    else
    {
        body_validation = smc_occp_validate_body(g_occp_data_buffer, count_len_bytes, body_crc_present);
        if (body_validation != NoErr)
        {
            simputs("Error in JUMP command body, Ignoring JUMP Command");
            smc_occp_handle_error_response(drv, drv_type, hdr, body_validation); // Body error
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_CMD_FAILED);
            return OCCP_ERROR_INVALID_COMMAND;
        }
        /* Extract address (8 bytes) in little-endian format as per spec */
        address = (uint64_t)g_occp_data_buffer[7] << 56 | (uint64_t)g_occp_data_buffer[6] << 48 |
                  (uint64_t)g_occp_data_buffer[5] << 40 | (uint64_t)g_occp_data_buffer[4] << 32 |
                  (uint64_t)g_occp_data_buffer[3] << 24 | (uint64_t)g_occp_data_buffer[2] << 16 |
                  (uint64_t)g_occp_data_buffer[1] << 8 | (uint64_t)g_occp_data_buffer[0];

        /* Validate manifest address before alignment */
        if (address == 0)
        {
            simputs("Manifest address cannot be NULL\n");
            smc_post_code_set_error(POST_CODE_ERROR_ACCESS);
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_VALIDATE_ADDRESS_FAILED);
            smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_Address);
            return OCCP_ERROR_ACCESS_VIOLATION;
        }

        if (address % 4 != 0)
        {
            simputs("Jump address is not 32-bit aligned\n");
            smc_post_code_set_error(POST_CODE_ERROR_ACCESS);
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_VALIDATE_ADDRESS_FAILED);
            smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_Address);
            return OCCP_ERROR_ACCESS_VIOLATION;
        }

        simputshex64("Manifest address (aligned): ", address);

        /* Check if manifest address access is allowed (respects current security mode) */
        ret = smc_occp_check_addr_access_allowed(address, sizeof(uint64_t), false); /* Minimum manifest header size */
        if (ret != OCCP_ERROR_NONE)
        {
            simputs("VALIDATE_AND_BOOT: Manifest address access denied\n");
            smc_post_code_set_error(POST_CODE_ERROR_ACCESS);
            smc_status_report(SMC_STATUS_TYPE_ERROR, SMC_OCCP_ERROR_VALIDATE_ADDRESS_FAILED);
            smc_occp_handle_error_response(drv, drv_type, hdr, Invalid_Address);
            return OCCP_ERROR_ACCESS_VIOLATION;
        }
        validate_response_header.body_crc_present = false; // No body CRC for validated response
        validate_response_header.i3c_flags = 0;            // Reserved
        validate_response_header.hdr.error = 0;            // No error
        validate_response_header.hdr.flags = hdr.flags;    // Echo back flags from request
        validate_response_header.hdr.app_id = hdr.app_id;  // Echo back app_id from request
        validate_response_header.hdr.msg_id = hdr.msg_id;  // Echo back msg_id from request
        validate_response_header.hdr.length = 0;           // No body for validated response
        // Calculate header CRC
        uint8_t *hdr_ptr = ((uint8_t *)&validate_response_header) + 1;
        validate_response_header.hdr_crc = calculate_crc8(hdr_ptr, 7);
        // Set response ready state before sending
        smc_post_code_set_occp_state(POST_CODE_OCCP_STATE_RESP_READY);

        smc_occp_send_to_bus(drv, drv_type, (uint8_t *)&validate_response_header, sizeof(validate_response_header), TRANSPORT_TIMEOUT);

        /*
         * Port: the ROM hands the manifest offset to the SEP through the
         * scratchpad and then halts in wfi. No SEP here; keep serving.
         */
        LOG_WRN("OCCP: AuthenticateImage manifest at 0x%016llx acknowledged, no SEP on this board",
                (unsigned long long)address);
    }
    return ret;
}

static uint64_t smc_occp_determine_i3c_address(uint8_t efuse_slot_id)
{
    /*
     * Port: the ROM reads a 64-bit provisional ID from eFuses, or falls back
     * to the chip-id straps. The Zephyr I3C target advertises the PID from
     * its devicetree node; this value is informational.
     */
    ARG_UNUSED(efuse_slot_id);
    uint64_t determined_pid = occp_port_i3c_pid();

    simputshex64("Determined I3C/I2C ID from Kconfig: ", determined_pid);
    return determined_pid;
}

static int smc_occp_init_i3c_channel(bool use_channel, uint8_t peripheral_controller_id, uint64_t i3c_id)
{
    int ret = OCCP_ERROR_NONE;
    if (use_channel)
    {
        /* Port: one Zephyr I3C target instance, selected by main.c. */
        extern const struct device *occp_target_i3c_device(void);
        extern uintptr_t occp_target_i3c_regs(void);

        int err = occp_link_i3c_init(&g_i3c_link, occp_target_i3c_device(), occp_target_i3c_regs());
        if (err == 0)
        {
            LOG_INF("I3C channel %u ready as OCCP target, pid 0x%012llx", peripheral_controller_id,
                    (unsigned long long)i3c_id);
            g_smc_active_interfaces.channel_drivers[g_smc_active_interfaces.num_channels] = &g_i3c_link;
            g_smc_active_interfaces.type[g_smc_active_interfaces.num_channels] = DRIVER_TYPE_I3C;
            g_smc_active_interfaces.num_channels++;
        }
        else
        {
            LOG_ERR("Failed to initialize I3C channel %u: %d", peripheral_controller_id, err);
            ret = OCCP_ERROR_INTERFACE_ERROR;
        }
    }
    return ret;
}

static int smc_occp_read_from_bus_4byte_aligned_or_complete_stream(interface_driver_t drv, driver_type_t drv_type, uint8_t *buffer, size_t length, uint32_t timeout, bool expect_excess_bytes, bool is_flush)
{
    /*
     * Port: both bus types deliver a whole transaction into the link
     * mailbox before the loop sees it, so the 4-byte alignment rule of the
     * Cadence RX FIFO and the timeout no longer apply.
     */
    ARG_UNUSED(drv_type);
    ARG_UNUSED(timeout);
    ARG_UNUSED(is_flush);
    return occp_link_read(drv, buffer, length, expect_excess_bytes);
}

static int smc_occp_send_to_bus(interface_driver_t drv, driver_type_t drv_type, const uint8_t *data, size_t length, uint32_t timeout)
{
    ARG_UNUSED(drv_type);
    ARG_UNUSED(timeout);
    simputshex16("Sending OCCP response with length: ", (uint16_t)length);
    return occp_link_send(drv, data, length);
}

static uint8_t smc_occp_check_addr_access_allowed(uint64_t addr, uint16_t access_size, bool is_write)
{
    /*
     * Port: the ROM protects its own data and stack and, in secure mode,
     * everything outside the OCCP SRAM window. Here the only reachable
     * memory is the emulated windows, so "inside a window, and writable
     * if writing" is the rule. The ROM image window is read-only.
     */
    if (access_size == 0)
    {
        simputs("Zero-length access not allowed\n");
        smc_post_code_set_error(POST_CODE_ERROR_ACCESS);
        return OCCP_ERROR_ACCESS_VIOLATION;
    }

    if (addr > (UINT64_MAX - access_size))
    {
        simputs("Address arithmetic would overflow\n");
        smc_post_code_set_error(POST_CODE_ERROR_ACCESS);
        return OCCP_ERROR_ACCESS_VIOLATION;
    }

    if (occp_mem_check(addr, access_size, is_write) != OCCP_ERROR_NONE)
    {
        LOG_WRN("OCCP: %s denied, 0x%016llx +%u is outside the emulated windows or read-only",
                is_write ? "write" : "read", (unsigned long long)addr, access_size);
        smc_post_code_set_error(POST_CODE_ERROR_ACCESS);
        return OCCP_ERROR_ACCESS_VIOLATION;
    }

    return OCCP_ERROR_NONE;
}

static int smc_occp_flush_interface_fifo(interface_driver_t drv, driver_type_t drv_type)
{
    ARG_UNUSED(drv_type);
    if (!drv)
    {
        return 0;
    }

    /*
     * Port: the ROM drains the hardware FIFO with a timeout. The mailbox
     * already holds the whole transaction, so drop what is left of it.
     */
    size_t flushed = occp_link_flush(drv);
    if (flushed != 0)
    {
        LOG_DBG("OCCP: flushed %u excess bytes", (unsigned int)flushed);
    }
    return (int)flushed;
}
