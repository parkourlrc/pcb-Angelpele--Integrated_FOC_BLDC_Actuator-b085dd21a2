//===================================================================================================//
//                                            MCP2551 CAN                                            //
//                                            Transceiver                                            //
//===================================================================================================//
#ifndef MCP2551_H
#define MCP2551_H

#include "esp32_S3.h"
#include <stddef.h>
#include <stdint.h>


#define CAN_Speed 1  //Mbps
#define CHUNK_PAYLOAD 7
#define RX_TIMEOUT_MS 50
#define TX_GAP_MS 2
#define MAX_CAN_PACKET_SIZE 105
#define CAN_RX_WAIT_MS 10


class CAN {
    public:
        int init();
        int test();
        template <typename T>
        bool sendData(const T& data, uint8_t ID);
        template <typename R>
        bool receiveData(R* packet, uint8_t expected_id, TickType_t timeout = pdMS_TO_TICKS(CAN_RX_WAIT_MS));
        void check_and_recover();

    private:
        void reset_rx_state();
        bool process_frame(const twai_message_t* msg);

        uint8_t  rx_buf[MAX_CAN_PACKET_SIZE];
        uint8_t  chunks_received = 0;
        uint8_t  expected_chunks = 0;
        uint16_t chunks_mask     = 0;
        uint32_t last_chunk_ms   = 0;
        uint32_t packets_received = 0;
};

#endif