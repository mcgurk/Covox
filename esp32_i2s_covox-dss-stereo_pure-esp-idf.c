/*
Component config -> ESP System settings:
 Also watch CPU1 tick interrupt: off (default on)
 Watch CPU1 Idle Task: off (default on)
Component config -> FreeRTOS -> Tick rate: 1000 (default 100)
Component config -> ESP32-specific -> CPU frequency: 240MHz (default: 160MHz)
*/
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

#define D0 13 // white
#define D1 14 // grey
#define D2 27 // yellow
#define D3 26 // brown
#define D4  9 // blue
#define D5 10 // purple
#define D6 18 // pink
#define D7 23 // green

#define I2S_BCK_IO      (GPIO_NUM_33) //4)
#define I2S_WS_IO       (GPIO_NUM_5)
#define I2S_DO_IO       (GPIO_NUM_32) //18)

#define FIFOCLK 19 // fifoclock, 17 (Select Printer_) (PC->DSS)
#define FIFOFULL 22 // fifofull, 10 (ACK) (DSS->PC)

#define STEREO_CHANNEL_SELECT 4
#define STEREO_CHANNEL_SELECT_PULLUP 25

#define GPIO_COVOX 2
#define GPIO_DSS 12
#define GPIO_STEREO 15

#define BOOL_COVOX (REG_READ(GPIO_IN_REG)&(1 << GPIO_COVOX))
#define BOOL_DSS (REG_READ(GPIO_IN_REG)&(1 << GPIO_DSS))
#define BOOL_STEREO (REG_READ(GPIO_IN_REG)&(1 << GPIO_STEREO))

#define SIZE_OF_DSS_BUF_IN_BYTES 256*4
#define CONVERT_GPIOREG_TO_SAMPLE(r) (uint8_t)((((r>>D0)&1)<<0) | (((r>>D1)&1)<<1) | (((r>>D2)&1)<<2) | (((r>>D3)&1)<<3) | (((r>>D4)&1)<<4) | (((r>>D5)&1)<<5) | (((r>>D6)&1)<<6) | (((r>>D7)&1)<<7))
TaskHandle_t myTaskHandle = NULL;

#define SAMPLE_RATE_DSS (14000)
#define SAMPLE_RATE_COVOX (96000)
#define I2S_NUM         (0)
#define I2S_DI_IO       (-1)

static const char* TAG = "mcgurk_DSS_system";

enum MODE { NONE = 0, COVOX = 1, DSS = 2, STEREO = 3 };
volatile uint32_t mode = NONE;

uint32_t buf[2048];
volatile uint32_t totalTaskCounter = 0;
volatile uint32_t totalSampleCounter = 0;
uint32_t totalSamplesPlayed = 0;
volatile uint8_t buffer_full = 0;
volatile uint8_t front = 0;
volatile uint8_t back = 0;
volatile uint32_t fifo_buf[256];
#define fcnt ((uint8_t)(back-front)) // 0-16
volatile uint32_t left, right;
volatile uint32_t last_dss_signal = 0;
volatile uint32_t last_stereo_signal = 0;
volatile uint32_t mode_change_flag = 0;
volatile uint32_t stereocount = 0;
volatile uint32_t debug = 0;

i2s_config_t i2s_config_covox = {
		.mode = I2S_MODE_MASTER | I2S_MODE_TX,
		.sample_rate = SAMPLE_RATE_COVOX,
		.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
		.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
		.communication_format = I2S_COMM_FORMAT_STAND_I2S,
		.dma_buf_count = 4,
		.dma_buf_len = 512,
		.use_apll = false,
		.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1                                //Interrupt level 1
};

i2s_config_t i2s_config_dss = {
		.mode = I2S_MODE_MASTER | I2S_MODE_TX,
		.sample_rate = SAMPLE_RATE_DSS,
		.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
		.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
		.communication_format = I2S_COMM_FORMAT_STAND_I2S,
		.dma_buf_count = 2,
		.dma_buf_len = 128,
		.use_apll = false,
		.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1                                //Interrupt level 1
};

i2s_pin_config_t pin_config = {
		.mck_io_num = I2S_PIN_NO_CHANGE,
		.bck_io_num = I2S_BCK_IO,
		.ws_io_num = I2S_WS_IO,
		.data_out_num = I2S_DO_IO,
		.data_in_num = I2S_DI_IO                                               //Not used
};

void covox_routine(void) {
	while(1) {
		register uint32_t a;
		do {
			a = REG_READ(GPIO_IN_REG);
			if ( !(a&(1 << GPIO_COVOX)) ) return;
		} while (!(a & (1<<I2S_WS_IO))); // a = when signal goes high
		uint32_t s1 = REG_READ(GPIO_IN_REG);
		uint32_t s2 = REG_READ(GPIO_IN_REG);
		uint32_t s3 = REG_READ(GPIO_IN_REG);
		if (s1 != s2) s1 = s3;
		uint16_t out = (CONVERT_GPIOREG_TO_SAMPLE(s1) - 128) << VOLUME;
		uint16_t i = totalSampleCounter & 2047;
		buf[i] = (out << 16) | out;
		if (i == 1023) buffer_full = 1;
		if (i == 2047) buffer_full = 2;
		totalSampleCounter++;
		totalTaskCounter++;
		while ((REG_READ(GPIO_IN_REG) & (1<<I2S_WS_IO))); // while signal is high
	}
}

void dss_routine(void) {
	while(1) {
		register uint32_t a;
		do {
			a = REG_READ(GPIO_IN_REG);
			if ( !(a&(1 << GPIO_DSS)) ) return;
		} while (!(a & (1<<FIFOCLK))); // a = when channel select goes high
		fifo_buf[back++] = a;
		//if ( !BOOL_DSS ) break;
		if (fcnt == 16) GPIO.out_w1ts = ((uint32_t)1 << FIFOFULL); //digitalWrite(FIFOFULL, HIGH);
		//while ((REG_READ(GPIO_IN_REG) & (1<<FIFOCLK))); // while fifoclk pin is high
		do {
			a = REG_READ(GPIO_IN_REG);
			if ( !(a&(1 << GPIO_DSS)) ) return;
		} while ((a & (1<<FIFOCLK))); // while fifoclk pin is high
		//totalTaskCounter++;
	}
}

void stereo_routine(void) {
	while(1) {
		while (1) {
			//do { a = REG_READ(GPIO_IN_REG); } while (!(a&(1<<STEREO_CHANNEL_SELECT))); // a = when channel select is high
			//do { b = REG_READ(GPIO_IN_REG); } while (b&(1<<STEREO_CHANNEL_SELECT)); // b = when channel select is low
			uint32_t temp_reg = 0, temp_reg2 = 0, temp_reg3 = 0;
			const uint32_t gpio_reg = 0x3FF4403C, mask = (1<<STEREO_CHANNEL_SELECT), endmask = (1<<GPIO_STEREO);
			const uint32_t left_ptr = (uint32_t)&left;
			const uint32_t right_ptr = (uint32_t)&right;
			__asm__ __volatile__(
					"loop1: \n"
					//"memw \n"
					"l32i.n	%0, %3, 0 \n" // read left channel
					"bnone	%0, %4, loop1 \n" // if LOW, go back to start
					"bnone    %0, %5, end \n" // enable bit low, quit
					//"memw \n"
					//"l32i.n	%2, %3, 0 \n"
					//"bnone	%2, %4, loop1 \n" // if LOW, go back to start
					" \n"
					"loop2: \n"
					//"memw \n"
					"l32i.n	%1, %3, 0 \n" // read right channel
					"bany 	%1, %4, loop2 \n" // if HIGH, go back to start
					//"memw \n"
					"l32i.n	%2, %3, 0 \n" //!why?
					"bany	  %2, %4, loop2 \n" //!why? // if HIGH, go back to start
					//"memw \n"
					"s32i.n	%0, %6, 0 \n"
					//"memw \n"
					"s32i.n	%1, %7, 0 \n"
					"j loop1 \n"
					"end: \n"
					: "=&r" (temp_reg), "=&r" (temp_reg2), "=&r" (temp_reg3) \
					  : "a" (gpio_reg), "a" (mask), "a" (endmask), "a" (left_ptr), "a" (right_ptr) );
			totalTaskCounter++;
			return;
		}
	}
}

void core1_task( void * pvParameters ) {
	portDISABLE_INTERRUPTS();
	while(1) {
		if ( BOOL_COVOX ) covox_routine();
		if ( BOOL_DSS ) dss_routine();
		if ( BOOL_STEREO ) stereo_routine();
		//debug++;
	}
	//vTaskDelay(10);
	//totalTaskCounter++;
}

void IRAM_ATTR isr_sample_stereo() {
	uint32_t l = left, r = right;
	uint16_t out_left = (CONVERT_GPIOREG_TO_SAMPLE(l)-128) << VOLUME;
	uint16_t out_right = (CONVERT_GPIOREG_TO_SAMPLE(r)-128) << VOLUME;
	uint16_t i = totalSampleCounter & 2047;
	buf[i] = (out_right << 16) | out_left;
	if (i == 1023) buffer_full = 1;
	if (i == 2047) buffer_full = 2;
	totalSampleCounter++;
}

void IRAM_ATTR isr_dssfifo() {
	static uint32_t out = 0;
	uint16_t i = totalSampleCounter & 255;
	if (i&1) {// read new "out" only every other time
		if (fcnt > 0) {
			uint32_t reg = fifo_buf[front++];
			uint16_t s = (CONVERT_GPIOREG_TO_SAMPLE(reg)-128) << VOLUME;
			out = (s << 16) | s;
		} else out = 0;
		if (fcnt < 16) GPIO.out_w1tc = ((uint32_t)1 << FIFOFULL); //digitalWrite(FIFOFULL, LOW);
	}
	buf[i] = out;
	if (i == 127) buffer_full = 1;
	if (i == 255) buffer_full = 2;
	totalSampleCounter++;
}

void IRAM_ATTR isr_stereo_detect() {
	//static uint32_t filter = 0;
	last_stereo_signal = esp_timer_get_time();
	stereocount++;
	//if (mode == COVOX) {
	/*if (mode != STEREO) {
		filter++;
		if (filter > 4) {
			mode_change_flag = STEREO;
			filter = 0;
		}
	} else filter = 0;*/
}

void IRAM_ATTR isr_dss_detect() {
	if ( !(REG_READ(GPIO_IN_REG)&(1<<FIFOCLK)) ) last_dss_signal = esp_timer_get_time();
	//if ( (REG_READ(GPIO_IN_REG)&(1<<FIFOCLK)) && (mode != DSS) ) mode_change_flag(DSS);
}

void change_mode(uint32_t new_mode) {
	uint32_t old_mode = mode;
	if (new_mode == old_mode) return;
	mode = NONE;

	switch (old_mode) { //disable current mode
	case COVOX:
		gpio_set_level(GPIO_COVOX, 0);
		i2s_driver_uninstall(I2S_NUM_0);
		break;
	case DSS:
		gpio_set_level(GPIO_DSS, 0);
		gpio_isr_handler_remove(I2S_WS_IO);
		i2s_driver_uninstall(I2S_NUM_0);
		break;
	case STEREO:
		gpio_set_level(STEREO_CHANNEL_SELECT_PULLUP, 0);
		gpio_set_level(GPIO_STEREO, 0);
		gpio_isr_handler_remove(I2S_WS_IO);
		i2s_driver_uninstall(I2S_NUM_0);
		break;
	default:
		break;
	}

	// clean up
	for (int i = 0; i < sizeof(buf)/sizeof(uint32_t); i++) buf[i] = 0;
	for (int i = 0; i < sizeof(fifo_buf)/sizeof(uint32_t); i++) fifo_buf[i] = 0;
	front = 0; back = 0;
	totalSampleCounter = 0;
	buffer_full = 0;

	switch (new_mode) {
	case COVOX:
		i2s_driver_install(I2S_NUM, &i2s_config_covox, 0, NULL);
		i2s_set_pin(I2S_NUM, &pin_config);
		gpio_set_level(GPIO_COVOX, 1);
		break;
	case DSS:
		i2s_driver_install(I2S_NUM, &i2s_config_dss, 0, NULL);
		i2s_set_pin(I2S_NUM, &pin_config);
		gpio_isr_handler_add(I2S_WS_IO, isr_dssfifo, NULL);
		gpio_set_level(GPIO_DSS, 1);
		break;
	case STEREO:
		gpio_set_level(STEREO_CHANNEL_SELECT_PULLUP, 1);
		i2s_driver_install(I2S_NUM, &i2s_config_covox, 0, NULL);
		i2s_set_pin(I2S_NUM, &pin_config);
		//pinMode(STEREO_CHANNEL_SELECT_PULLUP, OUTPUT); digitalWrite(STEREO_CHANNEL_SELECT_PULLUP, HIGH);
		gpio_isr_handler_add(I2S_WS_IO, isr_sample_stereo, NULL);
		gpio_set_level(GPIO_STEREO, 1);
		break;
	default:
		break;
	}

	gpio_hal_context_t gpiohal; gpiohal.dev = GPIO_LL_GET_HW(GPIO_PORT_0);
	gpio_hal_input_enable(&gpiohal, I2S_WS_IO);
	mode = new_mode;
	printf("New mode!: %i\n", mode);
}

#define PIN_TO_INPUT(pin) \
		gpio_reset_pin(pin); \
		gpio_pad_select_gpio(pin); \
		gpio_set_direction(pin, GPIO_MODE_INPUT); \
		gpio_pulldown_dis(pin); \
		gpio_pullup_dis(pin);

#define PIN_TO_OUTPUT(pin) \
		gpio_reset_pin(pin); \
		gpio_pad_select_gpio(pin); \
		gpio_set_direction(pin, GPIO_MODE_OUTPUT);

#define ms_to_cycles(a) 160000000L/1000L*a
#define cycles xthal_get_ccount

void app_main(void)
{

	PIN_TO_INPUT(D0); PIN_TO_INPUT(D1); PIN_TO_INPUT(D2); PIN_TO_INPUT(D3); PIN_TO_INPUT(D4); PIN_TO_INPUT(D5); PIN_TO_INPUT(D6); PIN_TO_INPUT(D7);
	PIN_TO_INPUT(FIFOCLK); PIN_TO_OUTPUT(FIFOFULL); gpio_set_level(FIFOFULL, 0);
	PIN_TO_INPUT(STEREO_CHANNEL_SELECT); PIN_TO_OUTPUT(STEREO_CHANNEL_SELECT_PULLUP); gpio_pulldown_en(STEREO_CHANNEL_SELECT); //gpio_set_level(STEREO_CHANNEL_SELECT_PULLUP, 1);
	PIN_TO_OUTPUT(GPIO_COVOX); gpio_set_level(GPIO_COVOX, 0);
	PIN_TO_OUTPUT(GPIO_DSS); gpio_set_level(GPIO_DSS, 0);
	PIN_TO_OUTPUT(GPIO_STEREO); gpio_set_level(GPIO_STEREO, 0);

	printf("setup running on core: %i\n", xPortGetCoreID());
	xTaskCreatePinnedToCore(core1_task, "Core1_Task", 4096, NULL,10, &myTaskHandle, 1);

	//ESP_LOGI(TAG, "set clock");
	//i2s_set_clk(I2S_NUM, SAMPLE_RATE, 16, 2);
	//ESP_LOGI(TAG, "write data");

	gpio_install_isr_service(0);
	gpio_set_intr_type(I2S_WS_IO, GPIO_INTR_POSEDGE);
	gpio_set_intr_type(FIFOCLK, GPIO_INTR_ANYEDGE);
	gpio_set_intr_type(STEREO_CHANNEL_SELECT, GPIO_INTR_POSEDGE);
	gpio_isr_handler_add(FIFOCLK, isr_dss_detect, NULL);
	gpio_isr_handler_add(STEREO_CHANNEL_SELECT, isr_stereo_detect, NULL);
	gpio_hal_context_t gpiohal; gpiohal.dev = GPIO_LL_GET_HW(GPIO_PORT_0);
	//gpio_hal_input_enable(&gpiohal, I2S_WS_IO);
	gpio_hal_input_enable(&gpiohal, GPIO_COVOX);
	gpio_hal_input_enable(&gpiohal, GPIO_DSS);
	gpio_hal_input_enable(&gpiohal, GPIO_STEREO);

	ESP_LOGI(TAG, "log test");

	change_mode(COVOX);
	//i2s_set_clk(I2S_NUM, SAMPLE_RATE_DSS, 16, 2);
	//mode = DSS;
	//i2s_driver_install(I2S_NUM, &i2s_config_covox, 0, NULL);
	//i2s_set_pin(I2S_NUM, &pin_config);
	//change_mode(COVOX);
	//vTaskDelay(1000 / portTICK_RATE_MS);

	while (1) {

		size_t i2s_bytes_write;
		if ((mode == DSS) && buffer_full) {
			//if (buffer_full) {
			if (buffer_full == 1) i2s_write(I2S_NUM_0, &buf[0], SIZE_OF_DSS_BUF_IN_BYTES/2, &i2s_bytes_write, portMAX_DELAY);
			if (buffer_full == 2) i2s_write(I2S_NUM_0, &buf[128], SIZE_OF_DSS_BUF_IN_BYTES/2, &i2s_bytes_write, portMAX_DELAY);
			buffer_full = 0;
			totalSamplesPlayed += i2s_bytes_write/4;
		}
		if ((mode != DSS) && buffer_full) {
			if (buffer_full == 1) i2s_write(I2S_NUM_0, &buf[0], 1024*4, &i2s_bytes_write, portMAX_DELAY);
			if (buffer_full == 2) i2s_write(I2S_NUM_0, &buf[1024], 1024*4, &i2s_bytes_write, portMAX_DELAY);
			buffer_full = 0;
			totalSamplesPlayed += i2s_bytes_write/4;
		}

		// debug:
		static uint32_t oldtime = 0, newtime = 0;
		//newtime = xthal_get_ccount();
		newtime = esp_timer_get_time();
		if ( (newtime - oldtime) < 2000000000L ) {
			rtc_cpu_freq_config_t conf;
			rtc_clk_cpu_freq_get_config(&conf);
			//printf("main core: %i, cpu speed: %u, cycles: %u, ",xPortGetCoreID(), conf.freq_mhz, xthal_get_ccount());
			//printf("main core: %i, cpu speed: %u, ",xPortGetCoreID(), conf.freq_mhz);
			printf("cpu speed: %u, ", conf.freq_mhz);
			//printf("debug: %u, ", debug);
			//printf("BOOL_COVOX: %u, ", BOOL_COVOX);
			printf("stereocount: %u, ", stereocount);
			printf("FIFOCLK: %u, ", (REG_READ(GPIO_IN_REG)>>FIFOCLK)&1);
			printf("esp_timer_get_time(): %u, ", newtime);
			printf("last_stereo: %u, ", newtime-last_stereo_signal);
			printf("last_dss: %u, ", newtime-last_dss_signal);
			//printf("stereo_detect_count: %u, ", stereo_detect_count);
			printf("mode: %u, ", mode);
			//printf("totalTaskCounter: %u, ", totalTaskCounter);
			//printf("totalSampleCounter: %u, ", totalSampleCounter);
			//printf("totalSamplesPlayed: %u, ", totalSamplesPlayed);
			//printf("difference: %u\n", totalSampleCounter-totalSamplesPlayed);
			printf("\n");
			oldtime += 1000000; //ms_to_cycles(1000);

		}

		if (mode_change_flag) {
			change_mode(mode_change_flag);
			mode_change_flag = NONE;
		}
		if (mode == STEREO) {
			uint32_t last = last_stereo_signal;
			uint32_t now = esp_timer_get_time();
			if ((now - last) > 1000000L) {
				change_mode(COVOX);
				//esp_restart();
			}
		}
		static uint32_t stereooldtime = 0, stereonewtime = 0;
		stereonewtime = esp_timer_get_time();
		if ( (stereonewtime - stereooldtime) < 2000000000L ) {
			if ( stereocount > 500 ) change_mode(STEREO); //100000, 30khz, > 3000
			stereooldtime += 100000; //400000, duke3d < 500
			stereocount = 0;
		}
		if ( (mode == DSS) && !(REG_READ(GPIO_IN_REG)&(1<<FIFOCLK)) ) {
			uint32_t last = last_dss_signal; // time when last falling edge happened
			uint32_t now = esp_timer_get_time();
			if ((now - last) > 100000L) // if FIFOCLK has been down 0,1s
				change_mode(COVOX);
		}
		if ( (mode != STEREO) && (REG_READ(GPIO_IN_REG)&(1<<FIFOCLK)) ) change_mode(DSS);

		vTaskDelay(1);
		//vTaskDelay(1000/portTICK_RATE_MS);

	}

}

/* LPT port pin 17 (fifoclk) to low
debug
n pin17.com
a100
mov dx,37a
in al,dx
or al,8             ; bit 4 high -> pin 17 low (and al,F7 to high)
out dx,al
ret

rcx
8
w
q
*/
