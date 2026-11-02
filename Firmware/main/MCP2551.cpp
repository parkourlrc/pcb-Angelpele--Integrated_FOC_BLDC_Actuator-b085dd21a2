//===================================================================================================//
//                                            MCP2551 CAN                                            //
//                                            Transceiver                                            //
//===================================================================================================//

#include "MCP2551.h"
#include <string.h>
#include <type_traits>

// Explicit instantiations for packet types defined in esp32_S3.h.
template bool CAN::sendData<TelemetryPacket>(const TelemetryPacket&, uint8_t);
template bool CAN::sendData<CANmotorPacket>(const CANmotorPacket&, uint8_t);
template bool CAN::sendData<CANMasterPacket>(const CANMasterPacket&, uint8_t);

template bool CAN::receiveData<TelemetryPacket>(TelemetryPacket*, uint8_t, TickType_t);
template bool CAN::receiveData<CANmotorPacket>(CANmotorPacket*, uint8_t, TickType_t);
template bool CAN::receiveData<CANMasterPacket>(CANMasterPacket*, uint8_t, TickType_t);

int CAN::init()
{
    gpio_set_direction(CAN_RX, GPIO_MODE_INPUT);
    gpio_set_direction(CAN_TX, GPIO_MODE_OUTPUT); 
        
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();      // Change filter to accept specific IDs

    if (twai_driver_install(&g, &t, &f) != ESP_OK) {
        printf("CAN INIT: Failed to install TWAI driver\n");
        return -1;
    }
    if (twai_start() != ESP_OK) {
        printf("CAN INIT: Failed to start TWAI driver\n");
        return -1;
    }
    return 0;
}

int CAN::test()
{
    CANmotorPacket test = {};
    test.ID = MOTOR_ID;
    test.status = MOTION_IDLE;
    test.mode = MODE_VEL;
    test.value_fbk = 1234.0f;

    // Send a test package using the actual motor-feedback packet format.
    if (!sendData(test, MOTOR_ID)) {
        printf("CAN TEST: TX failed\n");
        return -1;
    }
    return 0;
}

void CAN::check_and_recover()
{
    twai_status_info_t s = {};
    if (twai_get_status_info(&s) != ESP_OK) return;

    switch (s.state) {
        case TWAI_STATE_BUS_OFF:
            printf("CAN: Bus-Off detected. Initiating recovery...\n");
            twai_initiate_recovery();
            break;

        case TWAI_STATE_RECOVERING:
            break;

        case TWAI_STATE_STOPPED:
            printf("CAN: Recovery complete / stopped. Restarting TWAI...\n");
            reset_rx_state();
            twai_start();
            break;

        case TWAI_STATE_RUNNING:
            if (s.tx_error_counter > 128) {
                twai_clear_transmit_queue();
            }
            break;

        default:
            break;
    }
}


template <typename T>
bool CAN::sendData(const T& data, uint8_t ID)
{
    static_assert(std::is_trivially_copyable<T>::value, "CAN packets must be trivially copyable");
    check_and_recover();

    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&data);
    const size_t total = sizeof(T);
    if (total == 0 || total > MAX_CAN_PACKET_SIZE) return false;
    const uint8_t num_chunks = static_cast<uint8_t>((total + CHUNK_PAYLOAD - 1) / CHUNK_PAYLOAD);
    if (num_chunks > 15) return false;

    for (uint8_t i = 0; i < num_chunks; i++) {
        twai_message_t msg = {};
        msg.identifier       = ID;
        msg.extd             = 0;
        msg.rtr              = 0;
        msg.dlc_non_comp     = 0;
        msg.data_length_code = 8;

        size_t offset    = (size_t)i * CHUNK_PAYLOAD;
        size_t remaining = total - offset;
        size_t chunk_len = (remaining >= CHUNK_PAYLOAD) ? CHUNK_PAYLOAD : remaining;

        msg.data[0] = ((num_chunks & 0x0F) << 4) | (i & 0x0F);
        memset(&msg.data[1], 0, CHUNK_PAYLOAD);
        memcpy(&msg.data[1], raw + offset, chunk_len);

        if (twai_transmit(&msg, pdMS_TO_TICKS(20)) != ESP_OK) {
            twai_clear_transmit_queue(); // Discard the remainder of the broken burst
            check_and_recover();
            return false;
        }
    }
    return true;
}

// Clean up and reset state for new packet reception
void CAN::reset_rx_state() {
    memset(rx_buf, 0, sizeof(rx_buf));
    chunks_received = 0;
    expected_chunks = 0;
    chunks_mask     = 0;  
}


bool CAN::process_frame(const twai_message_t* msg)
{
    if (msg == nullptr || msg->data_length_code != 8 || msg->rtr) return false;

    uint8_t seq       = msg->data[0];
    uint8_t total     = (seq >> 4) & 0x0F;
    uint8_t chunk_idx = seq & 0x0F;

    // Reset received data if timeout since last chunk
    if (chunks_received > 0 && ((esp_timer_get_time() / 1000) - last_chunk_ms) > RX_TIMEOUT_MS)
        reset_rx_state();

    // chunk 0 always starts a new packet
    if (chunk_idx == 0) {
        reset_rx_state();
        expected_chunks = total;
    } else if (expected_chunks == 0) {
        return false; // missed chunk 0, wait for next burst
    }

    // Discard if sequence is inconsistent
    if (total == 0 || total > 15 || total != expected_chunks || chunk_idx >= total) {
        reset_rx_state();
        return false;
    }
    if (chunks_mask & (1U << chunk_idx)) return false;

    // Each frame contains seven data bytes; the final chunk is zero-padded.
    const size_t offset = (size_t)chunk_idx * CHUNK_PAYLOAD;
    if (offset + CHUNK_PAYLOAD > MAX_CAN_PACKET_SIZE) {
        reset_rx_state();
        return false;
    }

    memcpy(rx_buf + offset, &msg->data[1], CHUNK_PAYLOAD);

    chunks_mask |= (1 << chunk_idx);
    chunks_received++;
    last_chunk_ms = esp_timer_get_time() / 1000;  

    return (chunks_received == expected_chunks);    // Return true if packet is complete
}

template <typename R>
bool CAN::receiveData(R* packet, uint8_t expected_id, TickType_t timeout)
{
    static_assert(std::is_trivially_copyable<R>::value, "CAN packets must be trivially copyable");
    if (packet == nullptr || sizeof(R) > MAX_CAN_PACKET_SIZE) return false;

    const int64_t start_us = esp_timer_get_time();
    twai_message_t msg = {};

    while (true) {
        check_and_recover();

        TickType_t wait_ticks = timeout;
        if (timeout != portMAX_DELAY) {
            const int64_t elapsed_ms = (esp_timer_get_time() - start_us) / 1000;
            const int64_t timeout_ms = (int64_t)timeout * portTICK_PERIOD_MS;
            if (elapsed_ms >= timeout_ms) return false;
            const int64_t remaining_ms = timeout_ms - elapsed_ms;
            wait_ticks = pdMS_TO_TICKS((uint32_t)remaining_ms);
            if (wait_ticks == 0) wait_ticks = 1;
        }

        if (twai_receive(&msg, wait_ticks) != ESP_OK) {
            if (timeout == portMAX_DELAY) continue;
            return false;
        }

        if (msg.extd || msg.rtr || msg.identifier != expected_id) continue;

        if (process_frame(&msg)) {
            memcpy(packet, rx_buf, sizeof(R));
            reset_rx_state();
            return true;
        }
    }
}
