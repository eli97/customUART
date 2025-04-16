#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>

#define TX_PIN GPIO_NUM_16  
#define BAUD_RATE 9600      
#define BIT_TIME (1000000 / BAUD_RATE)  // Time for one bit in microseconds

void main() {
    // Configure TX pin as output
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << TX_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);

    // Set initial state to high (idle state)
    gpio_set_level(TX_PIN, 1);

    // Start UART task
    xTaskCreate(uart_task, "uart_task", 2048, NULL, 5, NULL);
}

void send_byte(uint8_t byte) {
    // Start bit
    send_bit(0);

    // Data bits (LSB first)
    for (int i = 0; i < 8; i++) {
        send_bit((byte >> i) & 1);
    }

    // Stop bit
    send_bit(1);
}

void uart_task(void *pvParameters) {
    while (1) {
        // Example: Send "Hello" repeatedly
        const char *message = "Hello\n";
        for (int i = 0; message[i] != '\0'; i++) {
            send_byte(message[i]);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay 1 second
    }
}

void send_bit(int bit) {
    gpio_set_level(TX_PIN, bit);
    ets_delay_us(BIT_TIME);  // Delay for one bit time
}