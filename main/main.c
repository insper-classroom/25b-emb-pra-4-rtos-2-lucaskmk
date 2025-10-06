#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pins.h"
#include "ssd1306.h" 

#define SOUND_SPEED_CM_US 0.0343f
#define MAX_ECHO_TIME_US 25000 
#define TIMEOUT_MS us_to_ms(MAX_ECHO_TIME_US + 5000)

ssd1306_t disp;

void pin_callback(uint gpio, uint32_t events);
void oled_display_init(void);
void hcsr04_init(void);
void led_rgb_init(void);
void ssd1306_draw_filled_rectangle(ssd1306_t *p, int x1, int y1, int x2, int y2);

void trigger_task(void *p);
void echo_task(void *p);
void oled_task(void *p);

SemaphoreHandle_t xSemaphoreTrigger; 
QueueHandle_t xQueueTime;          
QueueHandle_t xQueueDistance;      

static uint64_t start_time_us = 0;
static bool rising_edge_received = false;

void ssd1306_draw_filled_rectangle(ssd1306_t *p, int x1, int y1, int x2, int y2) {
    if (x1 > x2) { int temp = x1; x1 = x2; x2 = temp; }
    if (y1 > y2) { int temp = y1; y1 = y2; y2 = temp; }

    for (int x = x1; x <= x2; x++) {
        ssd1306_draw_line(p, x, y1, x, y2); 
    }
}

void oled_display_init(void) {
    i2c_init(i2c1, 400000);
    gpio_set_function(2, GPIO_FUNC_I2C);
    gpio_set_function(3, GPIO_FUNC_I2C);
    gpio_pull_up(2);
    gpio_pull_up(3);

    disp.external_vcc = false;
    ssd1306_init(&disp, 128, 64, 0x3C, i2c1);
    ssd1306_clear(&disp);
    ssd1306_show(&disp);
}

void led_rgb_init(void) {
    gpio_init(LED_PIN_R);
    gpio_set_dir(LED_PIN_R, GPIO_OUT);
    gpio_init(LED_PIN_G);
    gpio_set_dir(LED_PIN_G, GPIO_OUT);
    gpio_init(LED_PIN_B);
    gpio_set_dir(LED_PIN_B, GPIO_OUT);
    
    gpio_put(LED_PIN_R, 1);
    gpio_put(LED_PIN_G, 1);
    gpio_put(LED_PIN_B, 1);
}

void hcsr04_init(void) {
    gpio_init(HCSR04_PIN_TRIG);
    gpio_set_dir(HCSR04_PIN_TRIG, GPIO_OUT);
    gpio_put(HCSR04_PIN_TRIG, 0);

    gpio_init(HCSR04_PIN_ECHO);
    gpio_set_dir(HCSR04_PIN_ECHO, GPIO_IN);

    gpio_set_irq_enabled_with_callback(HCSR04_PIN_ECHO,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true, &pin_callback);
}

void pin_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (gpio == HCSR04_PIN_ECHO) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            start_time_us = time_us_64();
            rising_edge_received = true;

        } else if (events & GPIO_IRQ_EDGE_FALL) {
            if (rising_edge_received) {
                uint64_t end_time_us = time_us_64();
                uint32_t pulse_time_us = (uint32_t)(end_time_us - start_time_us);
                rising_edge_received = false; 

                xQueueSendFromISR(xQueueTime, &pulse_time_us, &xHigherPriorityTaskWoken);
            }
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void trigger_task(void *p) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(250);

    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        gpio_put(HCSR04_PIN_TRIG, 1);
        sleep_us(10);
        gpio_put(HCSR04_PIN_TRIG, 0);

        xSemaphoreGive(xSemaphoreTrigger);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void echo_task(void *p) {
    uint32_t pulse_time_us;
    float distance_cm;

    while (1) {
        if (xQueueReceive(xQueueTime, &pulse_time_us, portMAX_DELAY)) {

            distance_cm = (float)pulse_time_us * SOUND_SPEED_CM_US / 2.0f;

            xQueueSend(xQueueDistance, &distance_cm, 0);
        }
    }
}

void oled_task(void *p) {
    float distance_cm = -1.0f; 
    bool sensor_fail = true;
    char buffer[32];

    hcsr04_init();
    oled_display_init();
    led_rgb_init();

    while (1) {
        if (xSemaphoreTake(xSemaphoreTrigger, portMAX_DELAY)) {

            if (xQueueReceive(xQueueDistance, &distance_cm, pdMS_TO_TICKS(50))) {
                sensor_fail = false;

                if (distance_cm > 400.0f) {
                    distance_cm = 400.0f;
                }

            } else {
                sensor_fail = true;
                distance_cm = -1.0f;
            }

            gpio_put(LED_PIN_R, 1); 
            gpio_put(LED_PIN_G, 1);
            gpio_put(LED_PIN_B, 1);

            if (sensor_fail) {
                gpio_put(LED_PIN_R, 0); 
            } else if (distance_cm <= 100.0f) {
                gpio_put(LED_PIN_G, 0); 
            } else {
                gpio_put(LED_PIN_R, 0);
                gpio_put(LED_PIN_G, 0); 
            }

            ssd1306_clear(&disp);

            ssd1306_draw_string(&disp, 0, 0, 1, "HC-SR04 Distancia");

            if (sensor_fail) {
                ssd1306_draw_string(&disp, 0, 16, 2, "FALHA SENSOR");
                ssd1306_draw_string(&disp, 0, 32, 1, "------ cm");
            } else {
                snprintf(buffer, sizeof(buffer), "%.1f cm", distance_cm);
                ssd1306_draw_string(&disp, 0, 16, 2, buffer);

                int bar_width = (int)(distance_cm * 128.0f / 400.0f);
                if (bar_width > 127) bar_width = 127;

                ssd1306_draw_line(&disp, 0, 40, 127, 40);
                ssd1306_draw_line(&disp, 0, 50, 127, 50);
                ssd1306_draw_line(&disp, 0, 40, 0, 50);  
                ssd1306_draw_line(&disp, 127, 40, 127, 50);
                
                ssd1306_draw_filled_rectangle(&disp, 1, 41, bar_width, 49);

                int mark_1m = (int)(100.0f * 128.0f / 400.0f);
                ssd1306_draw_line(&disp, mark_1m, 40, mark_1m, 50);
            }
            
            ssd1306_show(&disp);
        }
    }
}

int main() {
    stdio_init_all();

    xQueueTime = xQueueCreate(1, sizeof(uint32_t));
    xQueueDistance = xQueueCreate(1, sizeof(float));
    xSemaphoreTrigger = xSemaphoreCreateBinary();

    xTaskCreate(echo_task, "Echo", 1024, NULL, 3, NULL);       
    xTaskCreate(trigger_task, "Trigger", 1024, NULL, 2, NULL); 
    xTaskCreate(oled_task, "OLED", 2048, NULL, 1, NULL);       

    vTaskStartScheduler();

    while (true);
}