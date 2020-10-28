/* SPI Slave example, sender (uses SPI master driver)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/igmp.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "soc/rtc_periph.h"
#include "esp32/rom/cache.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_spi_flash.h"

#include "driver/gpio.h"
#include "esp_intr_alloc.h"


/*
SPI sender (master) example.

This example is supposed to work together with the SPI receiver. It uses the standard SPI pins (MISO, MOSI, SCLK, CS) to 
transmit data over in a full-duplex fashion, that is, while the master puts data on the MOSI pin, the slave puts its own
data on the MISO pin.

This example uses one extra pin: GPIO_HANDSHAKE is used as a handshake pin. The slave makes this pin high as soon as it is
ready to receive/send data. This code connects this line to a GPIO interrupt which gives the rdySem semaphore. The main 
task waits for this semaphore to be given before queueing a transmission.
*/


/*
Pins in use. The SPI Master can use the GPIO mux, so feel free to change these if needed.
*/

// This pin is used on BW1092. On BW1098EMB it should connect to SPI2.5
// Assert low to put DepthAI bootloader in DFU update mode.
#define GPIO_UPDATE_DEPTHAI_FW 4
// Set to 1 only when updating DepthAI firmware, then reset to 0
#define ENABLE_FW_UPDATE 0

// DepthAI uses SPI2.6 pin as an interrupt output, open-drain, active low.
#define GPIO_HANDSHAKE 2 // ESP32 pin that connects to SPI2.6

#if 0 // original config in esp-idf example
#define GPIO_MOSI 12
#define GPIO_MISO 13
#define GPIO_SCLK 15
#define GPIO_CS 14
#else // GPIOs used in the upcoming board: DepthAI with ESP-WROOM-32
#define GPIO_MOSI 13
#define GPIO_MISO 12
#define GPIO_SCLK 14
#define GPIO_CS 15
#endif

#define SPI_PKT_SIZE 256

//The semaphore indicating the slave is ready to receive stuff.
static xQueueHandle rdySem;

/*
This ISR is called when the handshake line goes low.
*/
static void IRAM_ATTR gpio_handshake_isr_handler(void* arg)
{
    //Sometimes due to interference or ringing or something, we get two irqs after eachother. This is solved by
    //looking at the time between interrupts and refusing any interrupt too close to another one.
    static uint32_t lasthandshaketime;
    uint32_t currtime=xthal_get_ccount();
    uint32_t diff=currtime-lasthandshaketime;
    if (diff<240000) return; //ignore everything <1ms after an earlier irq
    lasthandshaketime=currtime;

    //Give the semaphore.
    BaseType_t mustYield=false;
    xSemaphoreGiveFromISR(rdySem, &mustYield);
    if (mustYield) portYIELD_FROM_ISR();
}

//Main application
void app_main()
{
    if (ENABLE_FW_UPDATE) {
        gpio_reset_pin    (GPIO_UPDATE_DEPTHAI_FW);
        gpio_set_direction(GPIO_UPDATE_DEPTHAI_FW, GPIO_MODE_OUTPUT);
        gpio_set_level    (GPIO_UPDATE_DEPTHAI_FW, 0);
    }

    esp_err_t ret;
    spi_device_handle_t handle;

    //Configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num=GPIO_MOSI,
        .miso_io_num=GPIO_MISO,
        .sclk_io_num=GPIO_SCLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };

    //Configuration for the SPI device on the other side of the bus
    spi_device_interface_config_t devcfg={
        .command_bits=0,
        .address_bits=0,
        .dummy_bits=0,
        .clock_speed_hz=100000, // TODO
        .duty_cycle_pos=128,        //50% duty cycle
        .mode=0,
        .spics_io_num=GPIO_CS,
        .cs_ena_posttrans=3,        //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size=3
    };

    //GPIO config for the handshake line.
    gpio_config_t io_conf={
        .intr_type=GPIO_PIN_INTR_NEGEDGE,
        .mode=GPIO_MODE_INPUT,
        .pull_up_en=1,
        .pin_bit_mask=(1<<GPIO_HANDSHAKE)
    };

    int n=0;
    static char sendbuf[SPI_PKT_SIZE] = {0};
    static char recvbuf[SPI_PKT_SIZE+1] = {0}; // +1 for extra null terminator
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    //Create the semaphore.
    rdySem=xSemaphoreCreateBinary();

    //Set up handshake line interrupt.
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_set_intr_type(GPIO_HANDSHAKE, GPIO_PIN_INTR_NEGEDGE);
    gpio_isr_handler_add(GPIO_HANDSHAKE, gpio_handshake_isr_handler, NULL);

    //Initialize the SPI bus and add the device we want to send stuff to.
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &handle);
    assert(ret==ESP_OK);

    //Assume the slave is ready for the first transmission: if the slave started up before us, we will not detect 
    //positive edge on the handshake line.
    xSemaphoreGive(rdySem);

    while(1) {
        int res = snprintf(sendbuf, sizeof(sendbuf), "esp32-cnt:%d", n);
        if (res >= sizeof(sendbuf)) {
            printf("Data truncated\n");
        }
        t.length=sizeof(sendbuf)*8;
        t.tx_buffer=sendbuf;
        t.rx_buffer=recvbuf;
        //Wait for slave to be ready for next byte before sending
#if 1 // Interrupt based
        xSemaphoreTake(rdySem, portMAX_DELAY); //Wait until slave is ready
#else // Polling / fixed wait before transfers, should still work, but no longer recommended
        vTaskDelay(30 / portTICK_PERIOD_MS);
#endif
        ret=spi_device_transmit(handle, &t);

        int index = 0;
        if (1) {
            // Sometimes it may happen that zero bytes are inserted at the beginning
            // of the received data, if the controller (ESP32) starts the transfer
            // before DepthAI prepared the buffers.
            // Workaround to try recovering the packet, by skipping zero bytes
            for (index = 0; index < SPI_PKT_SIZE; index++)
                if (recvbuf[index] != 0)
                    break;
        }
        char *spi_message = recvbuf + index;

        printf("[RECV-%d]", n);
        if (index == SPI_PKT_SIZE)
            printf(" (full-zero packet)");
        else if (index > 0)
            printf(" (skipped %d zero bytes)", index);
        printf(" %s\n", spi_message);

        // Parse SPI message string
        #define MAX_DETECTIONS 3
        int seq_num = -1;
        int num_detections = 0;
        static struct {
            int label;
            float score, x0, y0, x1, y1, dist_x, dist_y, dist_z;
        } det[MAX_DETECTIONS];
        char *current = spi_message;
        do {
            int bytes = 0;
            int elements = sscanf(current, "DepthAI %d\n%n", &seq_num, &bytes);
            if (seq_num < 0) break;
            for (int i = 0; i < MAX_DETECTIONS; i++) {
                int index = -1;
                current += bytes;
                elements = sscanf(current, "%d lbl %d %f%% xy0xy1 %f%f%f%f xyz %f%f%f\n%n",
                        &index, &det[i].label, &det[i].score,
                        &det[i].x0, &det[i].y0, &det[i].x1, &det[i].y1,
                        &det[i].dist_x, &det[i].dist_y, &det[i].dist_z, &bytes);
                if (index != i) break;
                if (elements <= 0) break;
                num_detections++;
            }
        } while (0);

        // Use parsed detections. Just print here for test
        for (int i = 0; i < num_detections; i++) {
            printf("[%d:%d] %2d %5.1f%% (%.3f,%.3f)->(%.3f,%.3f) %.2f %.2f %.2f\n",
                    seq_num, i, det[i].label, det[i].score,
                    det[i].x0, det[i].y0, det[i].x1, det[i].y1,
                    det[i].dist_x, det[i].dist_y, det[i].dist_z);
        }

        n++;
    }

    //Never reached.
    ret=spi_bus_remove_device(handle);
    assert(ret==ESP_OK);
}
