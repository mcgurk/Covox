/*
Covox/DSS/StereoIn1 implementation for ESP-IDF 5.0.2, ESP32-PICO-KIT and GY-PCM5102 by McGurk

https://dl.espressif.com/dl/esp-idf/
ESP-IDF v5.0.2 - Offline Installer (768MB) / esp-idf-tools-setup-offline-5.0.2.exe

https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2s.html
https://www.ti.com/lit/ds/symlink/pcm5102.pdf
https://github.com/espressif/esp-idf/blob/master/components/driver/i2s/include/driver/i2s_std.h
https://github.com/espressif/esp-idf/blob/master/components/driver/i2s/include/driver/i2s_common.h

New: Espressif IDF Project
Name: covox_simple
Target: ESP32
Component config -> ESP System settings:
 CPU frequency: 240MHz (default: 160MHz)
 Also watch CPU1 tick interrupt: off (default on)
 Watch CPU1 Idle Task: off (default on)
Component config -> FreeRTOS -> Tick rate: 1000 (default 100)
*/

#include "freertos/FreeRTOS.h" // task.h
#include "freertos/task.h" // vTaskDelay()
#include "soc/gpio_reg.h" // GPIO_IN_REG
#include "rom/gpio.h" // gpio_pad_select_gpio
#include "hal/gpio_hal.h" // gpio_hal_context_t
#include "esp_log.h" // ESP_LOGI
#include "esp_timer.h" // esp_timer_get_time
#include "soc/rtc.h" // rtc_cpu_freq_config_t
#include "driver/i2s_std.h" // I2S
#include "driver/gpio.h" // I2S

#define uint32_t unsigned int
#define int32_t int

#define VOLUME 8 // 0 min, 8 max
#define DEBUG

/* Pin definitions 														*/
/* --------------- 														*/
/* 	  Use only pins 0..31. those are in same register (GPIO_IN_REG) 	*/
/* 	  (* = bootstrap-pins can be used 									*/

#define D0 	(GPIO_NUM_18)	 // INPUT // brown
#define D1 	(GPIO_NUM_19)	 // INPUT // orange
#define D2 	(GPIO_NUM_27)	 // INPUT // blue
#define D3 	(GPIO_NUM_21)	 // INPUT // green
#define D4 	(GPIO_NUM_22)	 // INPUT // yellow
#define D5 	(GPIO_NUM_23)	 // INPUT // red
#define D6 	(GPIO_NUM_13)	 // INPUT // grey
#define D7 	(GPIO_NUM_14)	 // INPUT // purple

#define I2S_DO_IO       (GPIO_NUM_25)	// OUTPUT (*
#define I2S_BCK_IO      (GPIO_NUM_32) 	// OUTPUT (*
#define I2S_WS_IO       (GPIO_NUM_26)	// OUTPUT (* //MUST BE 0-31! (GPIO_IN_REG)

#define FIFOCLK 	(GPIO_NUM_9)	// INPUT	// white2 // fifoclock, 17 (Select Printer_), PC(LPT)->DSS(ESP32))
#define FIFOFULL 	(GPIO_NUM_10)	// OUTPUT	// black2 // fifofull, 10 (ACK), DSS(ESP32)->PC(LPT))

#define STEREO_CHANNEL_SELECT 			(GPIO_NUM_4)	//white1 // INPUT
//#define STEREO_CHANNEL_SELECT_PULLUP 	(GPIO_NUM_25)	// OUTPUT (*

#define GPIO_COVOX 		(GPIO_NUM_2)	// OUTPUT/INPUT (doesn't need physical pin)
#define GPIO_DSS 		(GPIO_NUM_12)	// OUTPUT/INPUT (doesn't need physical pin)
#define GPIO_STEREO 	(GPIO_NUM_15)	// OUTPUT/INPUT (doesn't need physical pin)

// GND // black1

/* Pin definition ends */

static const char* TAG = "McGurk_Covox/DSS/StereoIn1-system";

#define SIZE_OF_DSS_BUF_IN_BYTES 256*4
#define SAMPLE_RATE_DSS (14000)
#define SAMPLE_RATE_COVOX (96000)
//#define SAMPLE_RATE_COVOX (32000)

#define CONVERT_GPIOREG_TO_SAMPLE(r) (uint8_t)((((r>>D0)&1)<<0) | (((r>>D1)&1)<<1) | (((r>>D2)&1)<<2) | (((r>>D3)&1)<<3) | (((r>>D4)&1)<<4) | (((r>>D5)&1)<<5) | (((r>>D6)&1)<<6) | (((r>>D7)&1)<<7))
TaskHandle_t myTaskHandle = NULL;

enum MODE { NONE = 0, COVOX = 1, DSS = 2, STEREO = 3 };
static const char *MODE_STRING[] = { "NONE", "COVOX", "DSS", "STEREO" };
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
volatile uint32_t stereo_detect_count = 0;
volatile uint32_t stereocount = 0;
volatile uint8_t mode_flag = 0;
volatile uint8_t mode_routine = 0;
volatile uint32_t debug1 = 0;
//volatile uint32_t debug2 = 0;

i2s_chan_handle_t tx_handle;
i2s_std_config_t std_cfg = {
    .clk_cfg = {
        .sample_rate_hz = SAMPLE_RATE_COVOX,
        .clk_src = I2S_CLK_SRC_APLL,
        //.clk_src = I2S_CLK_SRC_DEFAULT,
        .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    },
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
        .mclk = I2S_GPIO_UNUSED, // @suppress("Symbol is not resolved")
        .bclk = I2S_BCK_IO,
        .ws = I2S_WS_IO,
        .dout = I2S_DO_IO,
        .din = I2S_GPIO_UNUSED, // @suppress("Symbol is not resolved")
        .invert_flags = {
            .mclk_inv = false,
            .bclk_inv = false,
            .ws_inv = false,
        },
    },
};

inline uint16_t get_sample_from_regvalue(uint32_t reg_value) __attribute__((always_inline)); // esp-idf 5.1 gives error without this?

inline uint16_t get_sample_from_regvalue(uint32_t reg_value) {
	uint8_t sample = CONVERT_GPIOREG_TO_SAMPLE(reg_value);
	if ( mode_flag == 0 ) return ( (sample - 128) << VOLUME );
	if ( sample == 0 ) return 0;
	mode_flag = 0;
	return ( (sample - 128) << VOLUME );
}

void covox_routine(void) {
	mode_routine = COVOX;
	while(1) {
		register uint32_t a;
		do {
			a = REG_READ(GPIO_IN_REG);
			if ( !(a&(1 << GPIO_COVOX)) ) return;
		} while (!(a & (1<<I2S_WS_IO))); // continue when I2S_WS signal goes HIGH
		uint32_t s1 = REG_READ(GPIO_IN_REG);
		uint32_t s2 = REG_READ(GPIO_IN_REG);
		uint32_t s3 = REG_READ(GPIO_IN_REG);
		if (s1 != s2) s1 = s3;
		uint16_t out = get_sample_from_regvalue(s1);
		uint16_t i = totalSampleCounter & 2047;
		buf[i] = (out << 16) | out;
		if (i == 1023) buffer_full = 1;
		if (i == 2047) buffer_full = 2;
		totalSampleCounter++;
		while ((REG_READ(GPIO_IN_REG) & (1<<I2S_WS_IO))); // wait while I2S_WS signal is HIGH. this syncronizes routine to I2S WS clock
	}
}

void dss_routine(void) {
	mode_routine = DSS;
	while(1) {
		register uint32_t a;
		do {
			a = REG_READ(GPIO_IN_REG);
			if ( !(a&(1 << GPIO_DSS)) ) return;
		} while (!(a & (1<<FIFOCLK))); // continue when FIFOCLK signal goes HIGH
		fifo_buf[back++] = a;
		if (fcnt == 16) GPIO.out_w1ts = ((uint32_t)1 << FIFOFULL); //digitalWrite(FIFOFULL, HIGH);
		do {
			a = REG_READ(GPIO_IN_REG);
			if ( !(a&(1 << GPIO_DSS)) ) return;
		} while ((a & (1<<FIFOCLK))); // continue when FIFOCLK signal goes LOW
		//totalSampleCounter++;
	}
}

void stereo_routine(void) {
	mode_routine = STEREO;
	while (1) {
		uint32_t temp_reg = 0, temp_reg2 = 0, temp_reg3 = 0;
		const uint32_t gpio_reg = 0x3FF4403C, mask = (1<<STEREO_CHANNEL_SELECT), endmask = (1<<GPIO_STEREO);
		const uint32_t combmask = mask | endmask;
		const uint32_t left_ptr = (uint32_t)&left;
		const uint32_t right_ptr = (uint32_t)&right;
		#ifdef DEBUG
		const uint32_t stereocount_ptr = (uint32_t)&stereocount;
		#endif
		__asm__ __volatile__(
			"loop1: \n"
			//"memw \n"
			"l32i.n	%[T1], %[GPIO], 0 \n" // read left channel
			//"bnone  %[T1], %[ENDMASK], end \n" // enable bit low, quit
			//"bnone	%[T1], %[MASK], loop1 \n" // if LOW, go back to start
			"bnone	%[T1], %[COMBMASK], loop1 \n" // if stereo signal and enable bit LOW, go back to start
			" \n"
			"bany   %[T1], %[ENDMASK], end \n" // enable bit high (inverted), quit
			"loop2: \n"
			//"memw \n"
			"l32i.n	%[T2], %[GPIO], 0 \n" // read right channel
			//"bnone  %[T2], %[ENDMASK], end \n" // enable bit low, quit // maybe not needed? this is needed!
			"bany   %[T2], %[ENDMASK], end \n" // enable bit high(inverted), quit // maybe not needed? this is needed!
			"bany 	%[T2], %[MASK], loop2 \n" // if HIGH, go back to start
			//"memw \n"
			"s32i.n	%[T1], %[LEFT], 0 \n" // store left channel
			//"memw \n"
			"s32i.n	%[T2], %[RIGHT], 0 \n" // store right channel
			// ----
			#ifdef DEBUG
			"l32i.n %[T1], %[COUNT], 0 \n" // inc stereocount
			"addi %[T1], %[T1], 1 \n"
			"s32i.n %[T1], %[COUNT], 0 \n"
			#endif
			// ----
			"j loop1 \n"
			"end: \n"
			: [T1]"=&r" (temp_reg), [T2]"=&r" (temp_reg2), [T3]"=&r" (temp_reg3) \
			: [GPIO]"a" (gpio_reg), [MASK]"a" (mask), [ENDMASK]"a" (endmask), [COMBMASK]"a" (combmask), [LEFT]"a" (left_ptr), [RIGHT]"a" (right_ptr) 
			#ifdef DEBUG
			, [COUNT]"a" (stereocount_ptr)
			#endif
		);
		//totalSampleCounter++;
		debug1 = 1;
		return;
	}
}

void core1_task( void * pvParameters ) {
	ESP_LOGI(TAG, "'core1_task' running on core: %i", xPortGetCoreID());
	portDISABLE_INTERRUPTS();
	while(1) {
		if ( gpio_get_level(GPIO_COVOX) ) covox_routine();
		if ( gpio_get_level(GPIO_DSS) ) dss_routine();
		if ( !gpio_get_level(GPIO_STEREO) ) stereo_routine(); // inverted
		totalTaskCounter++;  // not very meaningful anymore
	}
}

void IRAM_ATTR isr_sample_stereo() {
	uint32_t l = left, r = right;
	uint16_t out_left = get_sample_from_regvalue(l);
	uint16_t out_right = get_sample_from_regvalue(r);
	uint16_t i = totalSampleCounter & 2047;
	buf[i] = (out_right << 16) | out_left;
	if (i == 1023) buffer_full = 1;
	if (i == 2047) buffer_full = 2;
	totalSampleCounter++;
}

void IRAM_ATTR isr_dssfifo() {
	static uint32_t out = 0;
	uint16_t i = totalSampleCounter & 255;
	if (i&1) { // read new "out" only every other time
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
	last_stereo_signal = esp_timer_get_time();
	stereo_detect_count++;
}

void IRAM_ATTR isr_dss_detect() {
	if ( !(REG_READ(GPIO_IN_REG)&(1<<FIFOCLK)) ) last_dss_signal = esp_timer_get_time();
}

void change_mode(uint32_t new_mode) {
	uint32_t old_mode = mode;
	if (new_mode == old_mode) return;
	mode = NONE;

	i2s_channel_disable(tx_handle);

	switch (old_mode) { //disable current mode
	case COVOX:
		gpio_set_level(GPIO_COVOX, 0);
		break;
	case DSS:
		gpio_set_level(GPIO_DSS, 0);
		gpio_isr_handler_remove(I2S_WS_IO);
		break;
	case STEREO:
		gpio_set_level(GPIO_STEREO, 1); //inverted
		gpio_isr_handler_remove(I2S_WS_IO);
		break;
	default:
		break;
	}

	// clean up
	//for (int i = 0; i < sizeof(fifo_buf)/sizeof(uint32_t); i++) fifo_buf[i] = 0;
	front = 0; back = 0;
	totalSampleCounter = 0;
	totalSamplesPlayed = 0;
	stereocount = 0;
	buffer_full = 0;

	switch (new_mode) {
	case COVOX:
		std_cfg.clk_cfg.sample_rate_hz = SAMPLE_RATE_COVOX;
		i2s_channel_reconfig_std_clock(tx_handle, &std_cfg.clk_cfg);
		//gpio_hal_context_t gpiohal; gpiohal.dev = GPIO_LL_GET_HW(GPIO_PORT_0);
		//gpio_hal_input_enable(&gpiohal, I2S_WS_IO);
		gpio_set_level(GPIO_COVOX, 1);
		break;
	case DSS:
		std_cfg.clk_cfg.sample_rate_hz = SAMPLE_RATE_DSS;
		i2s_channel_reconfig_std_clock(tx_handle, &std_cfg.clk_cfg);
		gpio_isr_handler_add(I2S_WS_IO, isr_dssfifo, NULL);
		gpio_set_level(GPIO_DSS, 1);
		break;
	case STEREO:
		std_cfg.clk_cfg.sample_rate_hz = SAMPLE_RATE_COVOX;
		i2s_channel_reconfig_std_clock(tx_handle, &std_cfg.clk_cfg);
		gpio_isr_handler_add(I2S_WS_IO, isr_sample_stereo, NULL);
		gpio_set_level(GPIO_STEREO, 0); //inverted
		break;
	default:
		break;
	}

	i2s_channel_enable(tx_handle);
	mode = new_mode;
	mode_flag = 1;
	ESP_LOGI(TAG, "New mode: %s", MODE_STRING[mode]);
	if (mode_routine != mode) ESP_LOGW(TAG, "Routine mismatch! Running routine: %s", MODE_STRING[mode_routine]);
}

#define PIN_TO_INPUT(gpio_num) \
    { gpio_config_t cfg = { \
        .pin_bit_mask = BIT64(gpio_num), \
        .mode = GPIO_MODE_INPUT, \
        .pull_up_en = true, \
        .pull_down_en = false, \
        .intr_type = GPIO_INTR_DISABLE, \
    }; \
	gpio_config(&cfg); }

#define PIN_TO_OUTPUT(gpio_num) \
    { gpio_config_t cfg = { \
        .pin_bit_mask = BIT64(gpio_num), \
        .mode = GPIO_MODE_OUTPUT, \
        .pull_up_en = false, \
        .pull_down_en = false, \
        .intr_type = GPIO_INTR_DISABLE, \
    }; \
	gpio_config(&cfg); \
	gpio_set_level(gpio_num, 0); }

void app_main(void)
{
	esp_err_t result;

	ESP_LOGI(TAG, "Start");
	//ESP_LOGI(TAG, "Compilaton date: %s, time: %s", __DATE__, __TIME__);
	//ESP_LOGI(TAG, "ESP-IDF version: %s", IDF_VER);

	/* GPIO pins initialization */
	ESP_LOGI(TAG, "GPIO pins");
	ESP_LOGI(TAG, "Data 0...7:");
	PIN_TO_INPUT(D0); PIN_TO_INPUT(D1); PIN_TO_INPUT(D2); PIN_TO_INPUT(D3); PIN_TO_INPUT(D4); PIN_TO_INPUT(D5); PIN_TO_INPUT(D6); PIN_TO_INPUT(D7);
	ESP_LOGI(TAG, "Fifo clock:");
	PIN_TO_INPUT(FIFOCLK);
	ESP_LOGI(TAG, "Fifo full:");
	PIN_TO_OUTPUT(FIFOFULL);
	ESP_LOGI(TAG, "Stereo channel select:");
	PIN_TO_INPUT(STEREO_CHANNEL_SELECT);
	ESP_LOGI(TAG, "Mode (Covox, DSS, Stereo):");
	PIN_TO_OUTPUT(GPIO_COVOX);
	PIN_TO_OUTPUT(GPIO_DSS);
	PIN_TO_OUTPUT(GPIO_STEREO); gpio_set_level(GPIO_STEREO, 1); //inverted
	gpio_hal_context_t gpiohal; gpiohal.dev = GPIO_LL_GET_HW(GPIO_PORT_0);
	gpio_hal_input_enable(&gpiohal, GPIO_COVOX);
	gpio_hal_input_enable(&gpiohal, GPIO_DSS);
	gpio_hal_input_enable(&gpiohal, GPIO_STEREO);

	//printf("app_main running on core: %i\n", xPortGetCoreID());
	ESP_LOGI(TAG, "'app_main' running on core: %i", xPortGetCoreID());
	xTaskCreatePinnedToCore(core1_task, "Core1_Task", 4096, NULL,10, &myTaskHandle, 1);

	gpio_install_isr_service(0);
	gpio_set_intr_type(I2S_WS_IO, GPIO_INTR_POSEDGE);
	gpio_set_intr_type(FIFOCLK, GPIO_INTR_ANYEDGE);
	gpio_set_intr_type(STEREO_CHANNEL_SELECT, GPIO_INTR_POSEDGE);
	gpio_isr_handler_add(FIFOCLK, isr_dss_detect, NULL);
	gpio_isr_handler_add(STEREO_CHANNEL_SELECT, isr_stereo_detect, NULL);

	/* I2S initialization */
	i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
	chan_cfg.dma_desc_num = 4;
	chan_cfg.dma_frame_num = 512; // max 1023
	/* Allocate a new TX channel and get the handle of this channel */
	result = i2s_new_channel(&chan_cfg, &tx_handle, NULL);
	if (result != ESP_OK) printf("i2s_new_channel failed!");
	/* Initialize the channel */
	result = i2s_channel_init_std_mode(tx_handle, &std_cfg);
	if (result != ESP_OK) printf("i2s_channel init_std_mode failed!");
	/* Before writing data, start the TX channel first */
	result = i2s_channel_enable(tx_handle);
	if (result != ESP_OK) printf("i2s_channel_enable failed!");

	gpio_hal_input_enable(&gpiohal, I2S_WS_IO); // this makes possible to read I2S WS output-pin state

	change_mode(COVOX);

	while (1) {

		size_t i2s_bytes_write;
		if ((mode == DSS) && buffer_full) {
			if (buffer_full == 1) result = i2s_channel_write(tx_handle, &buf[0], SIZE_OF_DSS_BUF_IN_BYTES/2, &i2s_bytes_write, portMAX_DELAY);
			if (buffer_full == 2) result = i2s_channel_write(tx_handle, &buf[128], SIZE_OF_DSS_BUF_IN_BYTES/2, &i2s_bytes_write, portMAX_DELAY);
			if (result != ESP_OK) printf("i2s_channel_write (DSS) failed!");
			buffer_full = 0;
			totalSamplesPlayed += i2s_bytes_write/4;
		}
		if ((mode != DSS) && buffer_full) {
			if (buffer_full == 1) result = i2s_channel_write(tx_handle, &buf[0], 1024*4, &i2s_bytes_write, portMAX_DELAY);
			if (buffer_full == 2) result = i2s_channel_write(tx_handle, &buf[1024], 1024*4, &i2s_bytes_write, portMAX_DELAY);
			if (result != ESP_OK) printf("i2s_channel_write (Covox/StereoIn1) failed!");
			buffer_full = 0;
			totalSamplesPlayed += i2s_bytes_write/4;
		}

		#ifdef DEBUG
		static uint32_t oldtime = 0, newtime = 0;
		newtime = esp_timer_get_time();
		if ( (newtime - oldtime) < 2000000000L ) {
			rtc_cpu_freq_config_t conf;
			rtc_clk_cpu_freq_get_config(&conf);
			//printf("main core: %i, cpu speed: %u, cycles: %u, ",xPortGetCoreID(), conf.freq_mhz, xthal_get_ccount());
			//printf("main core: %i, cpu speed: %u, ",xPortGetCoreID(), conf.freq_mhz);
			//printf("cpu speed: %u, ", (unsigned int)conf.freq_mhz);
			//printf("BOOL_COVOX: %u, ", BOOL_COVOX);
			//printf("stereocount: %u, ", stereocount);
			//printf("FIFOCLK: %u, ", (REG_READ(GPIO_IN_REG)>>FIFOCLK)&1);
			printf("esp_timer_get_time(): %u, ", newtime);
			//printf("last_stereo: %u, ", newtime-last_stereo_signal);
			//printf("last_dss: %u, ", newtime-last_dss_signal);
			printf("stereocount: %u, ", stereocount);
			//printf("mode: %u, ", mode);
			printf("covox pin: %u, ", gpio_get_level(GPIO_COVOX));
			printf("Mode: %s, ", MODE_STRING[mode]);
			printf("totalTaskCounter: %u, ", totalTaskCounter);
			printf("totalSampleCounter: %u, ", totalSampleCounter);
			//printf("totalSamplesPlayed: %u, ", totalSamplesPlayed);
			//printf("difference: %u, ", ((int)totalSamplesPlayed)-((int)totalSampleCounter));
			//printf("difference: %i, ", ((int)totalSamplesPlayed)-((int)stereocount));
			//printf("mode_flag: %u, ", mode_flag);
			//printf("debug1: %u, ", debug1);
			printf("mode_routine: %u, ", mode_routine);
			printf("\n");
			oldtime += 1000000;
		}
		#endif

		if (mode_change_flag) {
			change_mode(mode_change_flag);
			mode_change_flag = NONE;
		}
		if (mode == STEREO) {
			uint32_t last = last_stereo_signal;
			uint32_t now = esp_timer_get_time();
			if ((now - last) > 1000000L) {
				change_mode(COVOX);
			}
		}
		static uint32_t stereooldtime = 0, stereonewtime = 0;
		stereonewtime = esp_timer_get_time();
		if ( (stereonewtime - stereooldtime) < 2000000000L ) {
			if ( stereo_detect_count > 500 ) change_mode(STEREO); //100000, 30khz, > 3000
			stereooldtime += 100000; //400000, duke3d < 500
			stereo_detect_count = 0;
		}
		if ( (mode == DSS) && !(REG_READ(GPIO_IN_REG)&(1<<FIFOCLK)) ) {
			uint32_t last = last_dss_signal; // time when last falling edge happened
			uint32_t now = esp_timer_get_time();
			if ((now - last) > 100000L) // if FIFOCLK has been down 0,1s
				change_mode(COVOX);
		}
		if ( (mode != STEREO) && (REG_READ(GPIO_IN_REG)&(1<<FIFOCLK)) ) change_mode(DSS);

		vTaskDelay(1);
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
