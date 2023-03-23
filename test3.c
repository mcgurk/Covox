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

//#define STEREO_CHANNEL_SELECT 4
//#define STEREO_CHANNEL_SELECT_PULLUP 25

#define GPIO_DSS 2

#define SIZE_OF_DSS_BUF_IN_BYTES 256*4
#define CONVERT_GPIOREG_TO_SAMPLE(r) (uint8_t)((((r>>D0)&1)<<0) | (((r>>D1)&1)<<1) | (((r>>D2)&1)<<2) | (((r>>D3)&1)<<3) | (((r>>D4)&1)<<4) | (((r>>D5)&1)<<5) | (((r>>D6)&1)<<6) | (((r>>D7)&1)<<7))
TaskHandle_t myTaskHandle = NULL;

#define SAMPLE_RATE_DSS (14000)
#define SAMPLE_RATE_COVOX (96000)
#define I2S_NUM         (0)
#define I2S_DI_IO       (-1)

static const char* TAG = "mcgurk_DSS_system";

enum MODE { NONE = 0, COVOX = 1, DSS = 2, STEREO = 3 };
enum MODE mode = NONE;

uint32_t buf[1024];
volatile uint32_t totalTaskCounter = 0;
volatile uint32_t totalSampleCounter = 0;
uint32_t totalSamplesPlayed = 0;
volatile uint8_t buffer_full = 0;
volatile uint8_t front = 0;
volatile uint8_t back = 0;
volatile uint32_t fifo_buf[256];
#define fcnt ((uint8_t)(back-front)) // 0-16
volatile uint8_t debug = 0;

#define BOOL_DSS (REG_READ(GPIO_IN_REG)&(1 << GPIO_DSS))

void dss_routine(void) {
	while(1) {
		register uint32_t a;
		do { a = REG_READ(GPIO_IN_REG); } while (!(a & (1<<FIFOCLK))); // a = when channel select goes high
		fifo_buf[back++] = a;
		if ( !BOOL_DSS ) break;
		if (fcnt == 16) GPIO.out_w1ts = ((uint32_t)1 << FIFOFULL); //digitalWrite(FIFOFULL, HIGH);
		while ((REG_READ(GPIO_IN_REG) & (1<<FIFOCLK))); // while fifoclk pin is high
		totalTaskCounter++;
	}
}

void core1_task( void * pvParameters ) {
	portDISABLE_INTERRUPTS();
	/*while(1) {
		if ( BOOL_DSS ) dss_routine();
	}*/
	//vTaskDelay(10);
	while (1) {
	register uint32_t a;
	do { a = REG_READ(GPIO_IN_REG); } while (!(a & (1<<FIFOCLK))); // a = when channel select goes high
	fifo_buf[back++] = a;
	//if ( !BOOL_DSS ) break;
	if (fcnt == 16) GPIO.out_w1ts = ((uint32_t)1 << FIFOFULL); //digitalWrite(FIFOFULL, HIGH);
	while ((REG_READ(GPIO_IN_REG) & (1<<FIFOCLK))); // while fifoclk pin is high
	totalTaskCounter++;
	}
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
	/*uint32_t reg = REG_READ(GPIO_IN_REG);
	uint16_t s = (CONVERT_GPIOREG_TO_SAMPLE(reg)-128) << VOLUME;
	uint16_t i = totalSampleCounter & 1023;
	out = (s << 16) | s;
	buf[i] = out;
	if (i == 511) buffer_full = 1;
	if (i == 1023) buffer_full = 2;*/
}


void change_mode(enum MODE new_mode) {
	enum MODE old_mode = mode;
	if (new_mode == old_mode) return;
	mode = NONE;

	for (int i = 0; i < sizeof(buf)/sizeof(uint32_t); i++) buf[i] = 0;
	for (int i = 0; i < sizeof(fifo_buf)/sizeof(uint32_t); i++) fifo_buf[i] = 0;
	front = 0; back = 0;
	totalSampleCounter = 0;
	buffer_full = 0;

	switch (old_mode) { //disable current mode
    	case COVOX:
    		//vTaskDelete(task_handle_covox); //disable task
    		//detachInterrupt(I2S_WS); //disable i2s interrupt
    		i2s_stop(I2S_NUM_0); //stop i2s
    		break;
    	case DSS:
    		gpio_set_level(GPIO_DSS, 0);
    		//vTaskDelete(task_handle_dss); //disable task
    		//detachInterrupt(I2S_WS); //disable i2s interrupt
    		i2s_stop(I2S_NUM_0); //stop i2s
    		break;
    	case STEREO:
    		//vTaskDelete(task_handle_stereo); //disable task
    		//detachInterrupt(I2S_WS); //disable i2s interrupt
    		i2s_stop(I2S_NUM_0); //stop i2s
    		break;
    	default:
    		break;
	}

	switch (new_mode) {
    	case COVOX: //dss -> covox
    		// I don't like this much... if I put COVOX to core0, DSS doesn't work after COVOX without crackling. so I must use core1 to COVOX:
    		//xTaskCreatePinnedToCore(core0_task_covox, "core0_task_covox", 4096, NULL, 5, &task_handle_covox, 1); // create task_handle_covox (creates I2S_WS interrupt)
    		i2s_set_sample_rates(I2S_NUM_0, SAMPLE_RATE_COVOX); // set sample rate
    		i2s_start(I2S_NUM_0);
    		break;
    	case DSS: // covox -> dss
    		i2s_set_sample_rates(I2S_NUM_0, SAMPLE_RATE_DSS); // set sample rate
    		i2s_start(I2S_NUM_0);
    		//xTaskCreatePinnedToCore(core0_task_dss, "core0_task_dss", 4096, NULL, 5, &task_handle_dss, 0); // create task_handle_dss
    		//attachInterrupt(I2S_WS, isr_sample_dss, RISING); // handles i2s samples
    		gpio_set_level(GPIO_DSS, 1);
    		break;
    	case STEREO:
    		i2s_set_sample_rates(I2S_NUM_0, SAMPLE_RATE_COVOX); // set sample rate
    		i2s_start(I2S_NUM_0);
    		//xTaskCreatePinnedToCore(core0_task_stereo, "core0_task_stereo", 4096, NULL, 5, &task_handle_stereo, 0);
    		//attachInterrupt(I2S_WS, isr_sample_stereo, RISING);
    		//pinMode(STEREO_CHANNEL_SELECT_PULLUP, OUTPUT); digitalWrite(STEREO_CHANNEL_SELECT_PULLUP, HIGH);
    		break;
    	default:
    		break;
	}

	mode = new_mode;
	printf("New mode!: %i", mode);
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

    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = SAMPLE_RATE_DSS,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S, //I2S_COMM_FORMAT_STAND_MSB,
        .dma_buf_count = 2,
        .dma_buf_len = 256,
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
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);

    printf("setup running on core: %i\n", xPortGetCoreID());
    xTaskCreatePinnedToCore(core1_task, "Core1_Task", 4096, NULL,10, &myTaskHandle, 1);

    //ESP_LOGI(TAG, "set clock");
    //i2s_set_clk(I2S_NUM, SAMPLE_RATE, 16, 2);
    //ESP_LOGI(TAG, "write data");
    //for (int i = 0; i< 100; i++) fifo_buf[i]=i;

	PIN_TO_INPUT(D0); PIN_TO_INPUT(D1); PIN_TO_INPUT(D2); PIN_TO_INPUT(D3); PIN_TO_INPUT(D4); PIN_TO_INPUT(D5); PIN_TO_INPUT(D6); PIN_TO_INPUT(D7);
	PIN_TO_INPUT(FIFOCLK); PIN_TO_OUTPUT(FIFOFULL); gpio_set_level(FIFOFULL, 0);
	//PIN_TO_OUTPUT(GPIO_DSS); gpio_set_level(GPIO_DSS, 0);

	gpio_install_isr_service(0);
	gpio_set_intr_type(I2S_WS_IO, GPIO_INTR_POSEDGE);
	gpio_isr_handler_add(I2S_WS_IO, isr_dssfifo, NULL);
    gpio_hal_context_t gpiohal; gpiohal.dev = GPIO_LL_GET_HW(GPIO_PORT_0);
    gpio_hal_input_enable(&gpiohal, I2S_WS_IO);
    //gpio_hal_input_enable(&gpiohal, GPIO_DSS);

    ESP_LOGI(TAG, "log test");

    //change_mode(DSS);
    //i2s_set_clk(I2S_NUM, SAMPLE_RATE_DSS, 16, 2);
    mode = DSS;

    while (1) {

		size_t i2s_bytes_write;
    	//if ((mode == DSS) & buffer_full) {
    	if (buffer_full) {
    		if (buffer_full == 1) i2s_write(I2S_NUM_0, &buf[0], SIZE_OF_DSS_BUF_IN_BYTES/2, &i2s_bytes_write, portMAX_DELAY);
    		if (buffer_full == 2) i2s_write(I2S_NUM_0, &buf[128], SIZE_OF_DSS_BUF_IN_BYTES/2, &i2s_bytes_write, portMAX_DELAY);
    	    buffer_full = 0;
    	    totalSamplesPlayed += i2s_bytes_write/4;
       	}
    	/*if ((mode != DSS) & buffer_full) {
    		if (buffer_full == 1) i2s_write(I2S_NUM_0, &buf[0], 512*4, &i2s_bytes_write, portMAX_DELAY);
    		if (buffer_full == 2) i2s_write(I2S_NUM_0, &buf[512], 512*4, &i2s_bytes_write, portMAX_DELAY);
    	    buffer_full = 0;
    	    totalSamplesPlayed += i2s_bytes_write/4;
       	}*/

    	// debug:
    	static uint32_t oldtime = 0, newtime = 0;
    	newtime = xthal_get_ccount();
    	if ( (newtime - oldtime) < 2000000000L ) {
        	rtc_cpu_freq_config_t conf;
            rtc_clk_cpu_freq_get_config(&conf);
    		printf("main core: %i, cpu speed: %u, cycles: %u, ",xPortGetCoreID(), conf.freq_mhz, xthal_get_ccount());
    		printf("totalTaskCounter: %u, ", totalTaskCounter);
    		printf("totalSampleCounter: %u, ", totalSampleCounter);
    		printf("totalSamplesPlayed: %u, ", totalSamplesPlayed);
    		printf("difference: %u\n", totalSampleCounter-totalSamplesPlayed);
			//for (int i = 0; i< 100; i++) printf("%i,", fifo_buf[i]);
			//for (int i = 0; i< 100; i++) printf("%i,", CONVERT_GPIOREG_TO_SAMPLE(fifo_buf[i]));
			//for (int i = 0; i< 20; i++) printf("%i,", buf[i]&255);
			//printf("\n");
    	    oldtime += ms_to_cycles(1000);

    	    /*static uint8_t test = 0;
    	    test ^= 0xff;
    	    gpio_set_level(BIT_DSS, test);
    	    printf("%i\n", debug);*/
    	}

    	//if (mode != STEREO) { // don't autodetect COVOX or DSS in STEREO-mode
    		/*static uint32_t dss_detect = 0;
    		//if (REG_READ(GPIO_IN_REG) & (1<<STEREO_CHANNEL_SELECT)) ESP.restart();
    		if (mode == DSS) if (REG_READ(GPIO_IN_REG) & (1<<FIFOCLK)) dss_detect = cycles();
    		if (mode == DSS) if ((cycles() - dss_detect) > ms_to_cycles(1000)) change_mode(COVOX);
    		if (mode != DSS) if (REG_READ(GPIO_IN_REG) & (1<<FIFOCLK)) change_mode(DSS);*/
    	//}

    	vTaskDelay(1);
    	//vTaskDelay(1000/portTICK_RATE_MS);

    }

}
