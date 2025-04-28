#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>

#define TX_PIN GPIO_NUM_16  // GPIO16 for TX (output)
#define RX_PIN GPIO_NUM_17  // GPIO17 for RX (input)
#define BAUD_RATE 9600
#define BIT_TIME_US (1000000 / BAUD_RATE)  // Bit time in microseconds (~104 us for 9600 baud)

static const char *TAG = "BITBANG_UART";

// Function to transmit a byte using bit-banging
void bitbang_uart_tx(uint8_t byte) {
    // Configure TX pin as output
    gpio_set_direction(TX_PIN, GPIO_MODE_OUTPUT);
    // Start bit (low)
    gpio_set_level(TX_PIN, 0);
    //ets_ delay_  us(BIT_TIME_US);

    // Send 8 data bits, LSB first
    for (int i = 0; i < 8; i++) {
        gpio_set_level(TX_PIN, (byte >> i) & 0x01);
        //ets_delay_us(BIT_TIME_US);
        esp_rom_delay_us(BIT_TIME_US);
    }

    // Stop bit (high)
    gpio_set_level(TX_PIN, 1);
    esp_rom_delay_us(BIT_TIME_US);
}

// Function to receive a byte using bit-banging
uint8_t bitbang_uart_rx(void) {
    // Configure RX pin as input
    gpio_set_direction(RX_PIN, GPIO_MODE_INPUT);
    uint8_t byte = 0;

    // Wait for start bit (low)
    while (gpio_get_level(RX_PIN) == 1) {
        //vTaskDelay(1 / portTICK_PERIOD_MS);  // Avoid busy-waiting too hard
    }

    // Delay to the middle of the start bit
    esp_rom_delay_us(BIT_TIME_US / 2);
    // Confirm start bit
    if (gpio_get_level(RX_PIN) != 0) {
      //ESP_LOGE(TAG, "Invalid start bit");
        return 0;
    }

    // Delay to the middle of the first data bit
    esp_rom_delay_us(BIT_TIME_US);
    // Read 8 data bits, LSB first
    for (int i = 0; i < 8; i++) {
        if (gpio_get_level(RX_PIN)) {
            byte |= (1 << i);
        }
        esp_rom_delay_us(BIT_TIME_US);
    }

    // Check stop bit (high)
    if (gpio_get_level(RX_PIN) != 1) {
       // ESP_LOG(TAG, "Invalid stop bit");
        return 0;
    }

    return byte;
}

void app_main(void) {
    // Configure GPIO pins
    gpio_reset_pin(TX_PIN);
    gpio_reset_pin(RX_PIN);
    gpio_set_direction(TX_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(RX_PIN, GPIO_MODE_INPUT);

    // Set TX pin to idle state (high)
    gpio_set_level(TX_PIN, 1);


    while (1) {
        // Test message to send
        const char *test_msg = "Hello, Bitbang!\n";
        int len = strlen(test_msg);
        //ESP_LOG(TAG, "Sending: %s", test_msg);

        // Transmit each character
        for (int i = 0; i < len; i++) {
            bitbang_uart_tx(test_msg[i]);
        }

        // Receive and log the data
        char received[32] = {0};
        int idx = 0;
        for (int i = 0; i < len; i++) {
            received[idx] = bitbang_uart_rx();
            idx++;
        }

        received[idx] = '\0';  // Null-terminate the string
        //ESP_LOG(TAG, "Received: %s", received);

        // Delay before the next transmission
       // vTaskDelay(2000 / portTICK_PERIOD_MS);

    }
  }
