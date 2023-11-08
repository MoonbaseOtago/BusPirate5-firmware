
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pirate.h"
#include "system_config.h"
#include "opt_args.h"
#include "hardware/uart.h"
#include "hiz.h"
#include "bio.h"	
#include "psu.h"
#include "pullups.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/irq.h"


const char *scope_pins(void)
{
	return "-\t-\t-\t-\t-\t-\t-\t-";
}

const char *scope_error(void){
	return t[T_MODE_ERROR_NO_EFFECT_HIZ];
}

void scope_settings(void)
{
	printf("Scope ()=()");
}
void scope_macro(uint32_t macro) {}
void scope_help(void)
{
		printf("st - trigger\r\n");
		printf("	a analog pin is the trigger pin\r\n");
		printf("	0-7 which digital pin is the trigger pin\r\n");
		printf("	v [0-9].[0-9] voltage level\r\n");
		printf("	+-nb  trigger on pos neg none both\r\n");
		printf("	^ voltage up 0.1\r\n");
		printf("	v voltage down 0.1\r\n");
		printf("	< move trigger towards start\r\n");
		printf("	> move trigger towards end\r\n");
		printf("\r\n");
		printf("sx - timebase\r\n");
		printf("	< move left\r\n");
		printf("	> move right\r\n");
		printf("	^ faster\r\n");
		printf("	v slower\r\n");
		printf("\r\n");
		printf("sy - y scale\r\n");
		printf("	+ increase scale\r\n");
		printf("	- decrease scale\r\n");
		printf("	^ move up\r\n");
		printf("	v move down\r\n");
		printf("\r\n");
		printf("sr - run\r\n");
		printf("	'' - same as last - button if stopped\r\n");
		printf("	1 o - once\r\n");
		printf("	n - normal\r\n");
		printf("	a - auto\r\n");
		printf("\r\n");
		printf("ss - stop - button if running\r\n");
}


void scope_cleanup(void)
{
}

uint32_t scope_periodic(void)
{
	static unsigned char down=0;
	if (gpio_get(EXT1)) {
		if (!down) {
			down = 1;
			//printf("down\r\n");
		}
	} else {
		if (down) {
			down = 0;
			//printf("up\r\n");
		}
	}
}

uint32_t scope_setup(void)
{
	return 1;
}

// this is called duringmode changes; takes care pwm, vpu and psu is turned off, also AUX to input
uint32_t scope_setup_exc(void)
{
	// turn everything off
	bio_init();     // make all pins safe
	psu_reset();    // disable psu and reset pin label
    psu_cleanup();  // clear any errors
	pullups_cleanup(); //deactivate
	system_config.freq_active=0;
	system_config.pwm_active=0;
	system_config.aux_active=0;
	for(int i=0;i<count_of(bio2bufiopin);i++)
	{
		system_bio_claim(false, i, BP_PIN_IO,0);
	}

	return 1;
}

#define CAPTURE_DEPTH 64
#define BUFFERS 32
static unsigned char buffer[2*BUFFERS*CAPTURE_DEPTH];
static int offset =0;
static uint dma_chan;
static unsigned char data_ready;
static unsigned char stop_capture;
static int int_count=0;

static void
dma_handler(void)
{
	int last_offset = offset;

	int_count++;
	dma_hw->ints0 = 1u << dma_chan;
	data_ready++;
	if (stop_capture) {
		if (stop_capture == 1) {
			stop_capture = 0;
			return;
		}
		stop_capture--;
	}
	offset += 2*CAPTURE_DEPTH;
	if (offset >= (2*CAPTURE_DEPTH*BUFFERS))
		offset = 0;
	dma_channel_set_write_addr(dma_chan, &buffer[offset], true);
	if (!stop_capture) {
		if (search_rising) {
			unsigned short *tp = (unsigned short *)&buffer[last_offset];
			for (int i = 0; i < CAPTURE_DEPTH; i++) {
				v = *tp++;
				if (last_value <= trigger_level && v > trigger_level) {
					stop_capture = BUFFERS-1;
					trigger_offset = i;
					break;
				}
				last_value = v;
			}
		} else 
		if (search_falling) {
			unsigned short *tp = (unsigned short *)&buffer[last_offset];
			for (int i = 0; i < CAPTURE_DEPTH; i++) {
				v = *tp++;
				if (last_value >= trigger_level && v < trigger_level) {
					stop_capture = BUFFERS-1;
					trigger_offset = i;
					break;
				}
				last_value = v;
		} else 
		if (search_either) {
			unsigned short *tp = (unsigned short *)&buffer[last_offset];
			for (int i = 0; i < CAPTURE_DEPTH; i++) {
				v = *tp++;
				if (last_value >= trigger_level ? v < trigger_level: v > trigger_level) {
					stop_capture = BUFFERS-1;
					trigger_offset = i;
					break;
				}
				last_value = v;
			}
		}
	}
}

static void
scope_stop(void)
{
	irq_set_enabled(DMA_IRQ_0, false);
	dma_channel_set_irq0_enabled(dma_chan, false);
	adc_run(false);
	dma_channel_abort(dma_chan);
    adc_fifo_drain();
	dma_channel_acknowledge_irq0(dma_chan);
	dma_hw->ints0 = 1u << dma_chan;
	dma_channel_cleanup(dma_chan);
	dma_channel_unclaim (dma_chan);
}

static void
scope_start(int pin)
{
printf("1\n\r");
	adc_init();
printf("2\n\r");
	adc_gpio_init(CURRENT_SENSE);
	adc_gpio_init(AMUX_OUT);
	adc_select_input(AMUX_OUT_ADC);
	hw_adc_channel_select(pin);
printf("6\n\r");

	adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        false     // Shift each sample to 8 bits when pushing to FIFO
    );
	adc_set_clkdiv(0);
printf("8\n\r");
	dma_chan = dma_claim_unused_channel(true);
	dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
	channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
	channel_config_set_read_increment(&cfg, false);
	channel_config_set_write_increment(&cfg, true);
	channel_config_set_dreq(&cfg, DREQ_ADC);
printf("9\n\r");
	dma_channel_set_irq0_enabled(dma_chan, true);
	irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
	irq_set_enabled(DMA_IRQ_0, true);
	dma_hw->ints0 = 1u << dma_chan;
printf("15\n\r");
	offset = 0;
	data_ready = 0;
	stop_capture = 0;
	dma_channel_configure(dma_chan, &cfg,
         &buffer[0],    // dst
        &adc_hw->fifo,  // src
        CAPTURE_DEPTH,  // transfer count
        true           // start 
    );
printf("18\n\r");
	
	printf("starting %d\r\n", int_count);
memset(buffer, 0, sizeof(buffer));
	busy_wait_ms(1);
	adc_run(true);

//dma_channel_wait_for_finish_blocking(dma_chan);
//printf("blocking done\r\n");
	for (int i = 0; i < 10000; i++) {
		if (data_ready) break;
		busy_wait_ms(1);
	}
	stop_capture = 1;
	printf("stopping %d\r\n", data_ready);
	for (int i = 0; i < 10000; i++) {
		if (!stop_capture) break;
		busy_wait_ms(1);
	}
	printf("stopped %d %d\r\n", stop_capture, int_count);
for (int i = 0; i < 10; i++) printf("%02x ", buffer[i]);printf("\r\n");

	scope_stop();
}


uint32_t
scope_commands(struct opt_args *args, struct command_result *result)
{
	printf("SCOPE: %s\r\n", args[0].c);
//
//	commands:
//
//	sx - scope X  up/down (scale) left right
//	sy - scope y
//  st - scope trigger +-nb up down left right ports- num 
//  sr - run [pin]
//  ss - stop 
//	
//
	if (strcmp(args[0].c, "st") == 0)  {
		// trigger
		// a 0-7 which is the trigger pin
		// v [0-9].[0-9] voltage level
		// +-nb  trigger on pos neg none both
		// ^ voltage up 0.1
		// v voltage down 0.1
		// < move trigger towards start
		// > move trigger towards end
		// p [0-1024] - trigger position
	} else
	if (strcmp(args[0].c, "sx") == 0)  {
		// x - timebase
		// < move left
		// > move right
		// ^ faster
		// v slower
	} else
	if (strcmp(args[0].c, "sy") == 0)  {
		// y - y scale
		// + increase scale
		// - decrease scale
		// ^ move up
		// v move down
	} else
	if (strcmp(args[0].c, "sr") == 0)  {
		// start 
		// <> same as last - button if stopped
		// 1 o - once
		// n - normal
		// a - auto
scope_start(2);
	} else 
	if (strcmp(args[0].c, "ss") == 0)  {
		// stop the engine - button if started
	} else {
		return 0;
	}
}

/* For Emacs:
 * Local Variables:
 * mode:c
 * indent-tabs-mode:t
 * tab-width:4
 * c-basic-offset:4
 * End: 
 * For VIM:
 * vim:set softtabstop=4 shiftwidth=4 tabstop=4:
 */ 
