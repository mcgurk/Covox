/*
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/
https://dl.espressif.com/dl/esp-idf/?idf=4.4
Espressif-IDE 2.9.1 with ESP-IDF v5.0.1 (1GB) / espressif-ide-setup-2.9.1-with-esp-idf-5.0.1.exe
New: Espressif IDF Project
Name: covox
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
#include "driver/i2s_std.h"
#include "driver/gpio.h"

#define uint32_t unsigned int
#define int32_t int

#define VOLUME 8 // 0 min, 8 max
#define DEBUG

#define D0 13 // white
#define D1 14 // grey
#define D2 27 // yellow
#define D3 26 // brown
#define D4  9 // blue
#define D5 10 // purple
#define D6 18 // pink
#define D7 23 // green

i2s_chan_handle_t tx_handle;
#define I2S_BCK_IO      (GPIO_NUM_33)
#define I2S_WS_IO       (GPIO_NUM_5)
#define I2S_DO_IO       (GPIO_NUM_32)

#define CONVERT_GPIOREG_TO_SAMPLE(r) (uint8_t)((((r>>D0)&1)<<0) | (((r>>D1)&1)<<1) | (((r>>D2)&1)<<2) | (((r>>D3)&1)<<3) | (((r>>D4)&1)<<4) | (((r>>D5)&1)<<5) | (((r>>D6)&1)<<6) | (((r>>D7)&1)<<7))

#define SAMPLE_RATE_COVOX (96000)

static const char* TAG = "mcgurk_Covox_system";

TaskHandle_t myTaskHandle = NULL;
uint32_t buf[2048];
volatile uint32_t totalTaskCounter = 0;
volatile uint32_t totalSampleCounter = 0;
uint32_t totalSamplesPlayed = 0;
volatile uint8_t buffer_full = 0;
volatile uint32_t left, right;

/* Setting the configurations, the slot configuration and clock configuration can be generated by the macros
 * These two helper macros are defined in 'i2s_std.h' which can only be used in STD mode.
 * They can help to specify the slot and clock configurations for initialization or updating */
i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE_COVOX),
    .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
        .mclk = I2S_GPIO_UNUSED,
        .bclk = I2S_BCK_IO,
        .ws = I2S_WS_IO,
        .dout = I2S_DO_IO,
        .din = I2S_GPIO_UNUSED,
        .invert_flags = {
            .mclk_inv = false,
            .bclk_inv = false,
            .ws_inv = false,
        },
    },
};

void core1_task( void * pvParameters ) {
	portDISABLE_INTERRUPTS();
	while(1) {
		register uint32_t a;
		do {
			a = REG_READ(GPIO_IN_REG);
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

#define PIN_TO_INPUT(pin) \
		gpio_reset_pin(pin); \
		gpio_pad_select_gpio(pin); \
		gpio_set_direction(pin, GPIO_MODE_INPUT); \
		gpio_pulldown_dis(pin); \
		gpio_pullup_dis(pin);

void app_main(void)
{

	PIN_TO_INPUT(D0); PIN_TO_INPUT(D1); PIN_TO_INPUT(D2); PIN_TO_INPUT(D3); PIN_TO_INPUT(D4); PIN_TO_INPUT(D5); PIN_TO_INPUT(D6); PIN_TO_INPUT(D7);

	/* Get the default channel configuration by the helper macro.
	 * This helper macro is defined in 'i2s_common.h' and shared by all the I2S communication modes.
	 * It can help to specify the I2S role and port ID */
	i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
	/* Allocate a new TX channel and get the handle of this channel */
	i2s_new_channel(&chan_cfg, &tx_handle, NULL);
	/* Initialize the channel */
	i2s_channel_init_std_mode(tx_handle, &std_cfg);
	/* Before writing data, start the TX channel first */
	i2s_channel_enable(tx_handle);

	/* If the configurations of slot or clock need to be updated, stop the channel first and then update it */
	// i2s_channel_disable(tx_handle);
	// std_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_MONO; // Default is stereo
	// i2s_channel_reconfig_std_slot(tx_handle, &std_cfg.slot_cfg);
	// std_cfg.clk_cfg.sample_rate_hz = 96000;
	// i2s_channel_reconfig_std_clock(tx_handle, &std_cfg.clk_cfg);
	// i2s_channel_enable(tx_handle);

	printf("setup running on core: %i\n", xPortGetCoreID());
	xTaskCreatePinnedToCore(core1_task, "Core1_Task", 4096, NULL,10, &myTaskHandle, 1);

	gpio_install_isr_service(0);
	gpio_set_intr_type(I2S_WS_IO, GPIO_INTR_POSEDGE);
	gpio_hal_context_t gpiohal; gpiohal.dev = GPIO_LL_GET_HW(GPIO_PORT_0);
	gpio_hal_input_enable(&gpiohal, I2S_WS_IO);

	ESP_LOGI(TAG, "log test");

	while (1) {

		size_t i2s_bytes_write;
		if (buffer_full) {
			if (buffer_full == 1) i2s_channel_write(tx_handle, &buf[0], 1024*4, &i2s_bytes_write, portMAX_DELAY); //check ESP_OK
			if (buffer_full == 2) i2s_channel_write(tx_handle, &buf[1024], 1024*4, &i2s_bytes_write, portMAX_DELAY); //check ESP_OK
			//if (buffer_full == 1) i2s_write(I2S_NUM_0, &buf[0], 1024*4, &i2s_bytes_write, portMAX_DELAY);
			//if (buffer_full == 2) i2s_write(I2S_NUM_0, &buf[1024], 1024*4, &i2s_bytes_write, portMAX_DELAY);
			buffer_full = 0;
			totalSamplesPlayed += i2s_bytes_write/4;
		}

		// debug:
		#ifdef DEBUG
		static uint32_t oldtime = 0, newtime = 0;
		newtime = esp_timer_get_time();
		if ( (newtime - oldtime) < 2000000000L ) {
			rtc_cpu_freq_config_t conf;
			rtc_clk_cpu_freq_get_config(&conf);
			//printf("main core: %i, cpu speed: %u, cycles: %u, ",xPortGetCoreID(), conf.freq_mhz, xthal_get_ccount());
			//printf("main core: %i, cpu speed: %u, ",xPortGetCoreID(), conf.freq_mhz);
			printf("cpu speed: %u, ", (unsigned int) conf.freq_mhz);
			printf("esp_timer_get_time(): %u, ", newtime);
			//printf("totalTaskCounter: %u, ", totalTaskCounter);
			//printf("totalSampleCounter: %u, ", totalSampleCounter);
			//printf("totalSamplesPlayed: %u, ", totalSamplesPlayed);
			//printf("difference: %u\n", totalSampleCounter-totalSamplesPlayed);
			printf("\n");
			oldtime += 1000000;
		}
		#endif

		vTaskDelay(1);
	}

}
