void app_main() {
  uart_init();
    uint8_t data[] = "Hi";
    send_packet(0x01, data, 2); // Send "Hi" with command 0x01
}

#include "driver/uart.h"
#include "soc/uart_reg.h"
#include "soc/uart_struct.h"
#include <string.h>

#define UART_PORT UART_NUM_2
#define TX_PIN 17
#define RX_PIN 16
#define BAUD_RATE 115200
#define BUFFER_SIZE 256

typedef enum {
  WAIT_START,
  WAIT_LENGTH,
  WAIT_COMMAND,
  WAIT_PAYLOAD,
  WAIT_CRC1,
  WAIT_CRC2,
  WAIT_END
} ParserState;

void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART parameters
    uart_param_config(UART_PORT, &uart_config);
    // Set UART pins
    uart_set_pin(UART_PORT, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Install UART driver with buffers
    uart_driver_install(UART_PORT, BUFFER_SIZE * 2, BUFFER_SIZE * 2, 0, NULL, 0);
}

// CRC16 implementation (using CCITT polynomial 0x1021)
uint16_t crc16(uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void send_packet(uint8_t command, uint8_t* payload, uint8_t payload_len) {
    uint8_t packet[260]; // Max packet: 1 + 1 + 1 + 255 + 2 + 1
    uint8_t idx = 0;

    // Build packet
    packet[idx++] = 0xAA;                    // Start byte
    packet[idx++] = payload_len;             // Length
    packet[idx++] = command;                 // Command
    memcpy(&packet[idx], payload, payload_len); // Payload
    idx += payload_len;
    uint16_t crc = crc16(&packet[1], payload_len + 2); // CRC over Length+Command+Payload
    packet[idx++] = crc >> 8;                // CRC high byte
    packet[idx++] = crc & 0xFF;              // CRC low byte
    packet[idx++] = 0xBB;                    // End byte

    // Send packet
    uart_write_bytes(UART_PORT, (const char*)packet, idx);
}

  
  void receive_packet(void) {
      static ParserState state = WAIT_START;
      static uint8_t buffer[260], payload_len = 0, command = 0, idx = 0;
      static uint16_t received_crc = 0;
      uint8_t byte;
  
      // Read one byte at a time
      while (uart_read_bytes(UART_PORT, &byte, 1, 0) > 0) {
          switch (state) {
              case WAIT_START:
                  if (byte == 0xAA) state = WAIT_LENGTH;
                  break;
              case WAIT_LENGTH:
                  payload_len = byte;
                  state = WAIT_COMMAND;
                  break;
              case WAIT_COMMAND:
                  command = byte;
                  idx = 0;
                  state = payload_len > 0 ? WAIT_PAYLOAD : WAIT_CRC1;
                  break;
              case WAIT_PAYLOAD:
                  buffer[idx++] = byte;
                  if (idx == payload_len) state = WAIT_CRC1;
                  break;
              case WAIT_CRC1:
                  received_crc = byte << 8;
                  state = WAIT_CRC2;
                  break;
              case WAIT_CRC2:
                  received_crc |= byte;
                  state = WAIT_END;
                  break;
              case WAIT_END:
                  if (byte == 0xBB) {
                      // Verify CRC
                      uint8_t crc_data[258];
                      crc_data[0] = payload_len;
                      crc_data[1] = command;
                      memcpy(&crc_data[2], buffer, payload_len);
                      if (crc16(crc_data, payload_len + 2) == received_crc) {
                          // Packet valid! Process it
                          // e.g., print buffer or handle command
                      }
                  }
                  state = WAIT_START; // Reset for next packet
                  break;
          }
      }
  }