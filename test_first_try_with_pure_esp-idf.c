#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include "soc/rtc.h"
#include "hal/gpio_hal.h"


#define VOLUME 4 // 0 min, 8 max
#define DEBUG
//#define EXTRA_GND 26

#define D0 4
#define D1 13
#define D2 14
#define D3 27
#define D4 9
#define D5 10
#define D6 18
#define D7 23
//#define FIFOFULL 10 // fifofull, 10 (ACK) (DSS->PC)
//#define FIFOCLK 9 // fifoclock, 17 (Select Printer_) (PC->DSS)
//#define STEREO_CHANNEL_SELECT 25

#define SIZE_OF_DSS_BUF_IN_BYTES 256*4
#define CONVERT_GPIOREG_TO_SAMPLE(r) (uint8_t)((((r>>D0)&1)<<0) | (((r>>D1)&1)<<1) | (((r>>D2)&1)<<2) | (((r>>D3)&1)<<3) | (((r>>D4)&1)<<4) | (((r>>D5)&1)<<5) | (((r>>D6)&1)<<6) | (((r>>D7)&1)<<7))
TaskHandle_t myTaskHandle = NULL;

#define SAMPLE_RATE     (50000)
#define I2S_NUM         (0)
#define I2S_BCK_IO      (GPIO_NUM_33) //4)
#define I2S_WS_IO       (GPIO_NUM_21) //(GPIO_NUM_5)
#define I2S_DO_IO       (GPIO_NUM_32) //18)
#define I2S_DI_IO       (-1)

static const char* TAG = "mcgurk_DSS_system";

uint32_t buf[1024];
volatile uint32_t totalTaskCounter = 0;
volatile uint32_t totalSampleCounter = 0;
uint32_t totalSamplesPlayed = 0;
volatile uint8_t buffer_full = 0;
//volatile uint8_t front = 0;
//volatile uint8_t back = 0;
volatile uint8_t fifo_buf[256];
//volatile uint32_t cycles;
//#define fcnt ((uint8_t)(back-front)) // 0-16

/*
    ESP_LOGI(TAG, "write data");
    i2s_write(I2S_NUM, samples_data, ((bits+8)/16)*SAMPLE_PER_CYCLE*4, &i2s_bytes_write, 100);

    free(samples_data);
}*/

void core1_task( void * pvParameters ) {
	while(1) {
		printf("core1_task running on core: %i\n", xPortGetCoreID());
		/*register uint32_t a;
		do { a = REG_READ(GPIO_IN_REG); } while (!(a & (1<<FIFOCLK))); // a = when channel select goes high
		fifo_buf[back++] = a;
		if (fcnt == 16) GPIO.out_w1ts = ((uint32_t)1 << FIFOFULL); //digitalWrite(FIFOFULL, HIGH);
		while ((REG_READ(GPIO_IN_REG) & (1<<FIFOCLK))); // while fifoclk pin is high
		totalTaskCounter++;*/
		vTaskDelay(100);
	}
}

void IRAM_ATTR isr_dssfifo() {
	static uint32_t out = 0;
	//uint16_t i = totalSampleCounter & 255;
	/*if (i&1) {// read new "out" only every other time
    if (fcnt > 0) {
      uint32_t g = fifo_buf[front++];
      uint16_t s = (CONVERT_GPIOREG_TO_SAMPLE(g)-128) << VOLUME;
      out = (s << 16) | s;
    } else out = 0;
    if (fcnt < 16) GPIO.out_w1tc = ((uint32_t)1 << FIFOFULL); //digitalWrite(FIFOFULL, LOW);
    }*/
	uint32_t reg = REG_READ(GPIO_IN_REG);
	uint16_t s = (CONVERT_GPIOREG_TO_SAMPLE(reg)-128) << VOLUME;
	uint16_t i = totalSampleCounter & 1023;
	out = (s << 16) | s;
	buf[i] = out;
	//if (i == 127) buffer_full = 1;
	//if (i == 255) buffer_full = 2;
	if (i == 511) buffer_full = 1;
	if (i == 1023) buffer_full = 2;
	totalSampleCounter++;
}


#define PIN_TO_INPUT(pin) \
	gpio_reset_pin(pin); \
	gpio_pad_select_gpio(pin); \
	gpio_set_direction(pin, GPIO_MODE_INPUT); \
    gpio_pulldown_dis(pin); \
    gpio_pullup_dis(pin);

void app_main(void)
{

    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S, //I2S_COMM_FORMAT_STAND_MSB,
        .dma_buf_count = 4,
        .dma_buf_len = 1024,
        .use_apll = false,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1                                //Interrupt level 1
    };
    i2s_pin_config_t pin_config = {
        .mck_io_num = 0, //I2S_PIN_NO_CHANGE,
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_DO_IO,
        .data_in_num = I2S_DI_IO                                               //Not used
    };
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);

    printf("setup running on core: %i\n", xPortGetCoreID());
    xTaskCreatePinnedToCore(core1_task, "Core1_Task", 4096, NULL,10, &myTaskHandle, 1);

    //ESP_LOGI(TAG, "set clock");
    //i2s_set_clk(I2S_NUM, SAMPLE_RATE, 16, 2);
    //ESP_LOGI(TAG, "write data");

	PIN_TO_INPUT(D0); PIN_TO_INPUT(D1); PIN_TO_INPUT(D2); PIN_TO_INPUT(D3); PIN_TO_INPUT(D4); PIN_TO_INPUT(D5); PIN_TO_INPUT(D6); PIN_TO_INPUT(D7);

	gpio_install_isr_service(0);
	gpio_set_intr_type(I2S_WS_IO, GPIO_INTR_POSEDGE);
	gpio_isr_handler_add(I2S_WS_IO, isr_dssfifo, NULL);
    gpio_hal_context_t gpiohal;
    gpiohal.dev = GPIO_LL_GET_HW(GPIO_PORT_0);
    gpio_hal_input_enable(&gpiohal, I2S_WS_IO);

    ESP_LOGI(TAG, "log test");

    while (1) {

    	if (buffer_full) {
    		size_t i2s_bytes_write;
    		//if (buffer_full == 1) i2s_write(I2S_NUM_0, &buf[0], SIZE_OF_DSS_BUF_IN_BYTES/2, &i2s_bytes_write, portMAX_DELAY);
    		//if (buffer_full == 2) i2s_write(I2S_NUM_0, &buf[128], SIZE_OF_DSS_BUF_IN_BYTES/2, &i2s_bytes_write, portMAX_DELAY);
    		if (buffer_full == 1) i2s_write(I2S_NUM_0, &buf[0], 512*4, &i2s_bytes_write, portMAX_DELAY);
    		if (buffer_full == 2) i2s_write(I2S_NUM_0, &buf[512], 512*4, &i2s_bytes_write, portMAX_DELAY);
    	    buffer_full = 0;
    	    totalSamplesPlayed += i2s_bytes_write/4;
    	}

    	// debug:
    	static uint32_t oldtime = 0, newtime = 0;
    	newtime = xthal_get_ccount();
    	if ( (newtime - oldtime) < 2000000000L ) { //newtime > oldtime ) {
        	rtc_cpu_freq_config_t conf;
            rtc_clk_cpu_freq_get_config(&conf);
    		printf("main core: %i, cpu speed: %u, cycles: %u, ",xPortGetCoreID(), conf.freq_mhz, xthal_get_ccount());
    		printf("totalSampleCounter: %u, ", totalSampleCounter);
    		printf("totalSamplesPlayed: %u, ", totalSamplesPlayed);
    		printf("difference: %u\n", totalSampleCounter-totalSamplesPlayed);
    	    oldtime += 240000000L;
    	}

    	vTaskDelay(1);
    	//vTaskDelay(1000/portTICK_RATE_MS);

    }

}
