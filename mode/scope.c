
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
#include "hardware/spi.h"
#include "font/font.h"
//#include "font/hunter-23pt-24h24w.h"
//#include "font/hunter-20pt-21h21w.h"
//#include "font/hunter-14pt-19h15w.h"
//#include "font/hunter-12pt-16h13w.h"
//#include "font/background.h"
//#include "font/background_image_v4.h"
extern const FONT_INFO hunter_12ptFontInfo;
#include "ui/ui_flags.h"
#include "ui/ui_lcd.h"
#include "ui/ui_prompt.h"
#include "ui/ui_parse.h"
#include "ui/ui_cmdln.h"
#include "usb_rx.h"

extern uint lcd_cs, lcd_dp;

#define VS 240
#define HS 320

#define CAPTURE_DEPTH 64
#define BUFFERS (5*10*2+1)	// 320x10 = standard samples rate (10 samples/pixel) 2x screen width
static uint16_t buffer[BUFFERS*CAPTURE_DEPTH];
static int offset =0;
static uint dma_chan;
static unsigned char data_ready;
static unsigned short stop_capture;
static int int_count=0;
static uint16_t last_value, trigger_level=1<<10;
static uint32_t trigger_offset;		// offset from start of dsplay in points
static uint32_t trigger_point;		// index of trigger point
static unsigned char search_rising, search_falling, search_either, first_sample;
static uint32_t h_res = 100;	// 100uS
static uint8_t scope_pin = 0;
static uint8_t display = 0;
static uint8_t running = 0;
static unsigned char fb[VS*HS];
typedef enum { SMODE_ONCE, SMODE_NORMAL, SMODE_AUTO } SCOPE_MODE;
SCOPE_MODE scope_mode=SMODE_NORMAL;
typedef enum { TRIGGER_POS, TRIGGER_NEG, TRIGGER_NONE, TRIGGER_BOTH } TRIGGER_TYPE;
TRIGGER_TYPE trigger_type = TRIGGER_BOTH;
static uint16_t dy=100;		// Y size in 10mV units
static uint16_t yoffset=0;  // y offset in dy units

static uint32_t timebase = 500000; // fastest possible
static uint16_t zoom=1;		// pixels/sample at the current timebase
static uint16_t samples=1;	// samples/pixel at the current timebase
static uint16_t xoffset=0;  // x offset in dy units

// b0   RRRr rGGG
// b1   gggB BBbb
unsigned short clr[] = 
{
	0x0000,	// black
	0xffff,	// white
	0x07ff, // yellow
	0x187f, // blue
	0x2086, // light green
	0x14a5, // gray
};

typedef enum {
BL=0,
WH=1,
Y=2,
B=3,
LG=4,
GY=5,
} CLR;

static void scope_start(int pin);
static void scope_stop(void);



const char *
scope_pins(void)
{
	return "-\t-\t-\t-\t-\t-\t-\t-";
}

const char *
scope_error(void)
{
	return t[T_MODE_ERROR_NO_EFFECT_HIZ];
}

void
scope_settings(void)
{
	printf("Scope ()=()");
}
void scope_macro(uint32_t macro) {}

void
scope_help(void)
{
		printf("t - trigger\r\n");
		printf("	a analog pin is the trigger pin\r\n");
		printf("	0-7 which digital pin is the trigger pin\r\n");
		printf("	v [0-9].[0-9] voltage level\r\n");
		printf("	+-nb  trigger on pos neg none both\r\n");
		printf("	^ voltage up 0.1\r\n");
		printf("	v voltage down 0.1\r\n");
		printf("	< move trigger towards start\r\n");
		printf("	> move trigger towards end\r\n");
		printf("\r\n");
		printf("x - timebase\r\n");
		printf("	< move left\r\n");
		printf("	> move right\r\n");
		printf("	^ faster\r\n");
		printf("	v slower\r\n");
		printf("\r\n");
		printf("y - y scale\r\n");
		printf("	+ increase scale\r\n");
		printf("	- decrease scale\r\n");
		printf("	^ move up\r\n");
		printf("	v move down\r\n");
		printf("\r\n");
		printf("sr - run\r\n");
		printf("	'' - same as last - button if stopped\r\n");
		printf("	0-7 - input pin\r\n");
		printf("	o - once\r\n");
		printf("	n - normal\r\n");
		printf("	a - auto\r\n");
		printf("\r\n");
		printf("ss - stop - button if running\r\n");
}


void scope_cleanup(void)
{
	ui_lcd_update(UI_UPDATE_ALL);
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
			if (!running) {
				scope_start(scope_pin);
			} else {
				scope_stop();
			}
		}
	}
}

uint32_t scope_setup(void)
{
	display = 1;
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
			display = 1;
			running = 0;
			return;
		}
		stop_capture--;
	}
	if (first_sample) {
		first_sample = 0;
		last_value = buffer[0];
	}
	offset += CAPTURE_DEPTH;
	if (offset >= (CAPTURE_DEPTH*BUFFERS))
		offset = 0;
	dma_channel_set_write_addr(dma_chan, &buffer[offset], true);
	if (!stop_capture) {
		if (search_rising) {
			unsigned short v, *tp = (unsigned short *)&buffer[last_offset];
			for (int i = 0; i < CAPTURE_DEPTH; i++) {
				v = *tp++;
				if (last_value <= trigger_level && v > trigger_level) {
					trigger_point = i;
					stop_capture = BUFFERS-1-(trigger_offset+CAPTURE_DEPTH-1)/CAPTURE_DEPTH;
					running = 0;
					break;
				}
				last_value = v;
			}
		} else 
		if (search_falling) {
			unsigned short v, *tp = (unsigned short *)&buffer[last_offset];
			for (int i = 0; i < CAPTURE_DEPTH; i++) {
				v = *tp++;
				if (last_value >= trigger_level && v < trigger_level) {
					trigger_point = i;
					stop_capture = BUFFERS-1-(trigger_offset+CAPTURE_DEPTH-1)/CAPTURE_DEPTH;
					running = 0;
					break;
				}
				last_value = v;
			}
		} else 
		if (search_either) {
			unsigned short v, *tp = (unsigned short *)&buffer[last_offset];
			for (int i = 0; i < CAPTURE_DEPTH; i++) {
				v = *tp++;
				if (last_value >= trigger_level ? v < trigger_level: v > trigger_level) {
					trigger_point = i;
					stop_capture = BUFFERS-1-(trigger_offset+CAPTURE_DEPTH-1)/CAPTURE_DEPTH;
					running = 0;
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
	display = 1;
}

static void
scope_start(int pin)
{

	search_falling = 1;
	search_either = 0;
	search_rising = 0;
	first_sample = 1;
	adc_init();
	adc_gpio_init(CURRENT_SENSE);
	adc_gpio_init(AMUX_OUT);
	adc_select_input(AMUX_OUT_ADC);
	hw_adc_channel_select(7-pin);

	adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        false     // Shift each sample to 8 bits when pushing to FIFO
    );
	switch (timebase) {
	case 500000: samples = 1; printf("clk=0\r\n"); adc_set_clkdiv(0); break;	
	case 250000: samples = 2; printf("clk=0\r\n"); adc_set_clkdiv(0); break;
	case 100000: samples = 5; printf("clk=0\r\n"); adc_set_clkdiv(0); break;
	case  50000: samples = 10; printf("clk=0\r\n"); adc_set_clkdiv(0); break;
//	case  10000: 
//	case   1000: 
//	case    100: 
//	case     10: samples = 5; printf("clk=%d\r\n", (5*9600000/5)/timebase - 1); adc_set_clkdiv((5*9600000/5)/timebase - 1); break;
	default:	 samples = 10; printf("clk=%d\r\n", (5*9600000/10)/timebase - 1); adc_set_clkdiv((5*9600000/10)/timebase - 1); break;
	}
	zoom = 1;
	//adc_set_clkdiv(0);
	dma_chan = dma_claim_unused_channel(true);
	dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
	channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
	channel_config_set_read_increment(&cfg, false);
	channel_config_set_write_increment(&cfg, true);
	channel_config_set_dreq(&cfg, DREQ_ADC);
	dma_channel_set_irq0_enabled(dma_chan, true);
	irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
	irq_set_enabled(DMA_IRQ_0, true);
	dma_hw->ints0 = 1u << dma_chan;
	offset = 0;
	data_ready = 0;
	stop_capture = 0;
	dma_channel_configure(dma_chan, &cfg,
         &buffer[0],    // dst
        &adc_hw->fifo,  // src
        CAPTURE_DEPTH,  // transfer count
        true           // start 
    );
	
	memset(buffer, 0, sizeof(buffer));
	display = 1;
	running = 1;
	busy_wait_ms(1);
	adc_run(true);

//dma_channel_wait_for_finish_blocking(dma_chan);
//printf("blocking done\r\n");
	for (int i = 0; i < 10000; i++) {
		if (data_ready) break;
		busy_wait_ms(1);
	}
	stop_capture = BUFFERS-5;
	printf("stopping %d\r\n", data_ready);
	for (int i = 0; i < 10000; i++) {
		if (!stop_capture) break;
		busy_wait_ms(1);
	}
	printf("stopped %d %d\r\n", stop_capture, int_count);
for (int i = 0; i < 40; i++) printf("%04x ", buffer[i]);printf("\r\n");

	scope_stop();
}

static uint8_t interactive;
char last;
static char 
next_char(void)
{
	char c;

	if (interactive) {
		for (;;) {
			if (rx_fifo_try_get(&c)) {
				if (last == 0x1b && c == 0x1b)
					return 0;
				if (c == 'x' || c == 'X'|| c == '\r')
					return 0;
				last = c;
				return c;
			}
			busy_wait_ms(10);
		}
	} else {
		if (!cmdln_try_peek(0, &c))
			return 0;	
		if (c == ';' || c == '&' || c == '|')
			return 0;
		cmdln_try_discard(1);
		return c;
	}
}

static void
trigger_up(void)
{
	printf("up\r\n");
}

static void
trigger_down(void)
{
	printf("down\r\n");
}

static void
trigger_left(void)
{
	printf("left\r\n");
}

static void
trigger_right(void)
{
	printf("right\r\n");
}

static void
scope_up(void)
{
	if ((yoffset+5)*dy < 500) {
		yoffset++;
		display = 1;
	}
}

static void
scope_down(void)
{
	if (yoffset > 0) {
		yoffset--;
		display = 1;
	}
}

static void
scope_left(void)
{
	printf("left\r\n");
}

static void
scope_right(void)
{
	printf("right\r\n");
}



uint32_t
scope_commands(struct opt_args *args, struct command_result *result)
{
	char c;
	//printf("SCOPE: %s\r\n", args[0].c);
	
//
//	commands:
//
//	x - scope X  timebase
//	y - scope y  voltage scaling
//  t - scope trigger +-nb up down left right ports- num 
//  sr - run [pin]
//  ss - stop 
//	
//
	last = 0;
	for (;;) {
		interactive = !cmdln_try_peek(0,&c);
		if (interactive)
			break;
		if (c == ' ' || c== '\t') {
			cmdln_try_discard(1);
			continue;
		}
		break;
	}
	
	if (strcmp(args[0].c, "t") == 0)  {
		// trigger
		// a 0-7 which is the trigger pin
		// v [0-9].[0-9] voltage level
		// +-nb  trigger on pos neg none both
		// ^ voltage up 0.1
		// v voltage down 0.1
		// < move trigger towards start
		// > move trigger towards end
		// p [0-1024] - trigger position
		printf("Trigger: a 0-7 +-nb ^v<> x> ");
		while ((c = next_char()) != 0) {
			switch (c) {
			case 0x1b:
				c = next_char();
				if (c != '[')
					break;
				c = next_char();
				switch (c) {
				case 'A': trigger_up();	break;
				case 'B': trigger_down();	break;
				case 'C': trigger_right();break;
				case 'D': trigger_left();	break;
				}
				break;
			case '=':
			case '+':	trigger_type = TRIGGER_POS; break;
			case '_':
			case '-':	trigger_type = TRIGGER_NEG; break;
			case 'n':	trigger_type = TRIGGER_NONE; break;
			case 'b':	trigger_type = TRIGGER_BOTH; break;
			case 'v':
			case 'V':	trigger_down(); break;
			case '^':	trigger_up(); break;
			case '<':	trigger_left(); break;
			case '>':	trigger_right(); break;
			}
		}
		return 1;
	} else
	if (strcmp(args[0].c, "x") == 0)  {
		int z;
		// x - timebase
		// < move left
		// > move right
		// + faster
		// - slower
		printf("Timebase: +- ^v<> x> ");
		while ((c = next_char()) != 0) {
			switch (c) {
			case 0x1b:
				c = next_char();
				if (c != '[')
					break;
				c = next_char();
				switch (c) {
				case 'A': scope_up();	break;
				case 'B': scope_down();	break;
				case 'C': scope_right();break;
				case 'D': scope_left();	break;
				}
				break;
			case '=':
			case '+':
				if (timebase == 500000)
					break;
				switch (timebase) {
				case 250000: timebase = 500000; z = 1; break;	// 1 sample
				case 100000: timebase = 250000; z = 0; break;	// 2 samples
				case  50000: timebase = 100000; z = 1; break;	// 5 samples
				case  25000: timebase = 50000; z = 1; break;	
				case  10000: timebase = 25000; z = 0; break;
				case   5000: timebase = 10000; z = 1; break;
				case   2500: timebase = 5000; z = 1; break;
				case   1000: timebase = 2500; z = 0; break;
				case    500: timebase = 1000; z = 1; break;
				case    250: timebase = 500; z = 1; break;
				case    100: timebase = 250; z = 0; break;
				case     50: timebase = 100; z = 1; break;
				case     25: timebase = 50; z = 1; break;
				case     10: timebase = 25; z = 0; break;
				}
				if (z) {
					if (samples != 1) {
						samples /= 2;
					} else {
						zoom *= 2;
					}
				} else {
					if (samples != 1) {
						samples *= 2;
						samples /= 5;
					} else {
						zoom *= 5;
						zoom /= 2;
					}
				}
printf("+ %dz=%d zoom=%d samples=%d\r\n", timebase, z, zoom, samples);
				display = 1;
				break;
			case '_':
			case '-':
				if (timebase == 10)
					break;
				switch (timebase) {
				case 500000: timebase = 250000; z = 1; break;
				case 250000: timebase = 100000; z = 0; break;
				case 100000: timebase = 50000; z = 1; break;
				case  50000: timebase = 25000; z = 1; break;
				case  25000: timebase = 10000; z = 0; break;
				case  10000: timebase = 5000; z = 1; break;
				case   5000: timebase = 2500; z = 1; break;
				case   2500: timebase = 1000; z = 0; break;
				case   1000: timebase = 500; z = 1; break;
				case    500: timebase = 250; z = 1; break;
				case    250: timebase = 100; z = 0; break;
				case    100: timebase = 50; z = 1; break;
				case     50: timebase = 25; z = 1; break;
				case     25: timebase = 10; z = 0; break;
				}
				if (z) {
					if (zoom == 1) {
						samples *= 2;
					} else {
						zoom /= 2;
					}
				} else {
					if (zoom == 1) {
						samples *= 5;
						samples /= 2;
					} else {
						zoom *= 2;
						zoom /= 5;
					}
				}
printf("- %d z=%d zoom=%d samples=%d\r\n", timebase, z, zoom, samples);
				display = 1;
				break;
			case 'v':
			case 'V':	scope_down(); break;
			case '^':	scope_up(); break;
			case '<':	scope_left(); break;
			case '>':	scope_right(); break;
			}
		}
		return 1;
	} else
	if (strcmp(args[0].c, "y") == 0)  {
		// y - y scale
		// + increase scale
		// - decrease scale
		// ^ move up
		// v move down
		printf("Voltage scale: +- ^v<> x> ");
		while ((c = next_char()) != 0) {
			switch (c) {
			case 0x1b:
				c = next_char();
				if (c != '[')
					break;
				c = next_char();
				switch (c) {
				case 'A': scope_up();	break;
				case 'B': scope_down();	break;
				case 'C': scope_right();break;
				case 'D': scope_left();	break;
				}
				break;
			case '=':
			case '+':	if (dy > 5) {
							switch (dy) {
							case 100: dy = 50; yoffset = 2*yoffset; break;
							case 50: dy = 20; yoffset = 5*yoffset/2; break;
							case 20: dy = 10; yoffset = 2*yoffset; break;
							case 10: dy = 5; yoffset = 2*yoffset; break;
							}
							if ((yoffset+5)*dy >= 500) {
								yoffset = 500/dy-5;
							}  
							display = 1;
						}
						break;
			case '_':
			case '-':	if (dy < 100) {
							switch (dy) {
							case 50: dy = 100; yoffset /= 2; break;
							case 20: dy = 50; yoffset = yoffset*2/5; break;
							case 10: dy = 20; yoffset = yoffset/2; break;
							case 5: dy = 10; yoffset = yoffset/2; break;
							}
							if ((yoffset+5)*dy >= 500) {
								yoffset = 500/dy-5;
							}  
							display = 1;
						}
						break;
			case 'v':
			case 'V':	scope_down(); break;
			case '^':	scope_up(); break;
			case '<':	scope_left(); break;
			case '>':	scope_right(); break;
			}
		}
		return 1;
	} else
	if (strcmp(args[0].c, "sr") == 0)  {
		// start 
		// <> same as last - button if stopped
		// 0-7 - pin
		// o - once
		// n - normal
		// a - auto
		for (;;) {
			char c;
			if (!cmdln_try_peek(0, &c))
				break;
			cmdln_try_discard(1);
			if (c == 0)
				break;
			switch (c) {
			case '0':
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
				scope_pin = c-'0';
				break;
			case 'o':
			case 'O':
				scope_mode = SMODE_ONCE;
				break;
			case 'n':
			case 'N':
				scope_mode = SMODE_NORMAL;
				break;
			case 'a':
			case 'A':
				scope_mode = SMODE_AUTO;
				break;
			case ' ':
				break;
			default:
				printf("invalid start mode '%c'\r\n", c);
				return 0;
			}
		}
		scope_start(scope_pin);
	} else 
	if (strcmp(args[0].c, "ss") == 0)  {
		// stop the engine - button if started
		scope_stop();
	} else {
		return 0;
	}
}
static void
scope_fb_init() 
{
	memset(fb, BL, sizeof(fb));
}


static void
draw_h_line(int x1, int x2, int y1, CLR c)
{
	unsigned char *p = &fb[y1*HS+x1];
	for (int i = x1; i <= x2; i++)
		*p++ = c;
}
static void
draw_v_line(int x1, int y1, int y2, CLR c)
{
	unsigned char *p = &fb[y1*HS+x1];
	for (int i = y1; i <= y2; i++) {
		*p = c;
		p += HS;
	}
}

static void
draw_grid(CLR c)
{
	scope_fb_init();
	for (int i = 0; i < 5 ; i++) 
		draw_h_line(0, HS-1, i*(VS/5), c);
	draw_h_line(0, HS-1, VS-1, c);
	for (int i = 0; i < 7 ; i++) 
		draw_v_line(i*50, 0, VS-1, c);
	//draw_v_line(HS-1, 0, VS-1, c);
}

static void 
draw_text(int x, int y, const FONT_INFO *font, CLR c, char *cp) // stolen from lcd_write_string
{
	unsigned char *b = &fb[x];
	int yHS = HS*y;
	while (*cp) {
		uint8_t adjusted_c = (*cp)-(*font).start_char;
		for (uint16_t col=0; col < (*font).lookup[adjusted_c].width; col++) {
			uint16_t row=0;
			uint16_t rows = (*font).lookup[adjusted_c].height;
			uint16_t offset=(*font).lookup[adjusted_c].offset;
			int off = yHS;
			for (uint16_t page=0; page<(*font).height_bytes; page++) {
				uint8_t bitmap_char = (*font).bitmaps[offset+(col*(*font).height_bytes)+page];
				for (uint8_t i=0; i<8; i++) {
					if (off >= 0 && bitmap_char&(0x80>>i)) {
						b[off] = c;
					} 
					off -= HS;
					row++;
					if(row==rows)
						break;
				}
			}
			b++;
		}
		cp++;
	}
}

static void
draw_trace(CLR c)
{
	unsigned char *p = &fb[0];
	uint16_t *b = &buffer[0];
	const uint32_t V5 = 0x0c05; // 5V
	int16_t v1[HS];
	int16_t v2[HS];
	// dy is the number of 10 mv per 
	// yoffset is the offest from 0 in dy units
	int df = yoffset*(VS/5);
printf("samples=%d zoom=%d\r\n", samples, zoom);
	for (int x = 0; x < HS; ) {
		for (int i=0; i < samples; i++) {
			uint32_t y = *b++;
		
			int d = (y*VS*100/V5/dy)-df;
//if (x<2) printf("y=%d dy=%d yoffset=%d df=%d d=%d\r\n", (int)y, (int)dy, (int)yoffset, df, d);
			if (d < 0) {
				d = -1;
			} else {
				if (d >=VS) {
					d = VS;
				}
			}
			if (i == 0) {
				v1[x] = d;
				v2[x] = d;
			} else {
				if (d < v1[x])
					v1[x] = d;
				if (d > v2[x])
					v2[x] = d;
			}
		}
		x += zoom;
	}
	int y1 = v1[0];
	int y2 = v2[0];
	if (v1[1] > y2) {
		y2 = (y2+v1[1])/2;
	} else 
	if (v2[1] < y1) {
		y1 = (y1+v2[1])/2;
	}
	if ((y1 >= 0 && y1 < VS) || (y2 >= 0 && y2 < VS)) {
		if (y1 >= VS) y1 = VS-1; 
		if (y2 >= VS) y2 = VS-1;
		if (y1 < 0) y1 = 0;
		if (y2 < 0) y2 = 0;
		if (y1 > y2) {
			int t = y1;
			y1 = y2;
			y2 = t;
		}
		for (int y = y1; y <= y2; y++)
			p[HS*y+0] = c;
	}
	for (int x=1; x < (HS-1); x++) {
		y1 = v1[x];
		y2 = v2[x];

		if (v1[x+1] > y2) {
			if (v1[x-1] > v1[x+1]) {
				y2 = (y2+v1[x-1]);
			} else {
				y2 = (y2+v1[x+1]);
			}
		} else
		if (v1[x-1] > y2) {
			y2 = (y2+v1[x-1]);
		} else {
			y2 = y2<<1;
		}

		if (v2[x+1] < y1) {
			if (v2[x-1] < v2[x+1]) {
				y1 = (y1+v2[x-1]);
			} else {
				y1 = (y1+v2[x+1]);
			}
		} else
		if (v2[x-1] < y1) {
			y1 = (y1+v2[x-1]);
		} else {
			y1 = y1<<1;
		} 

		//y1 = (v[x-1]+v[x]);
		//y2 = (v[x+1]+v[x]);
		if (y1 >= (2*VS)) {
			if (y2 >= (2*VS))
				continue;
			y1 = 2*(VS-1);
		} else
		if (y2 >= (2*VS)) {
			y2 = 2*(VS-1);
		} 

		if (y1 < 0) {
			if (y2 < 0)
				continue;
			y1 = 0;
		} else
		if (y2 < 0) {
			y2 = 0;
		} 

		//if (y1 > y2) {
		//	int t = y1;
		//	y1 = y2;
		//	y2 = t;
		//}
		for (int y = y1&~1; y <= y2; y+=2)
			p[(HS/2)*y+x] = c;
	}
	y1 = v1[HS-1];
	y2 = v2[HS-1];
	if (v1[HS-2] > y2) {
		y2 = (y2+v1[HS-2])/2;
	} 
	if (v2[HS-2] < y1) {
		y1 = (y1+v2[HS-2])/2;
	}
	if ((y1 >= 0 && y1 < VS) || (y2 >= 0 &&y2 < VS)) {
		if (y1 >= VS) y1 = VS-1;
		if (y2 >= VS) y2 = VS-1;
		if (y1 < 0) y1 = 0;
		if (y2 < 0) y2 = 0;
		if (y1 > y2) {
			int t = y1;
			y1 = y2;
			y2 = t;
		}
		for (int y = y1; y <= y2; y++)
			p[HS*y+HS-1] = c;
	}
}

static int
xnum(char *cp, int v)
{
	if (v == 0) return 0;
	int i = xnum(cp, v/10);
	cp += i;
	v = v%10;
	*cp = v+'0';
	return i+1;
}

static void 
draw_scope()
{
	char b[40];
	char *s1, *s2;	
	int unit;
	draw_grid(Y);
	
	switch (dy) {
	case 100: s1="1Vx";    break;
	case 50:  s1="0.5Vx";  break;
	case 20:  s1="0.2Vx";  break;
	case 10:  s1="0.1Vx";  break;
	case 5:   s1="0.05Vx"; break;
	}	
	switch (timebase) {
	case 500000: s2 = "100uS"; unit = 1; break;
	case 250000: s2 = "200uS"; unit = 2; break;
	case 100000: s2 = "500uS"; unit = 5; break;
	case  50000: s2 = "1mS"; unit = 10; break;
	case  25000: s2 = "2mS"; unit = 20; break;
	case  10000: s2 = "5mS"; unit = 50; break;
	case   5000: s2 = "10mS"; unit = 100; break;
	case   2500: s2 = "20mS"; unit = 200; break;
	case   1000: s2 = "50mS"; unit = 500; break;
	case    500: s2 = "100mS"; unit = 1000; break;
	case    250: s2 = "200mS"; unit = 2000; break;
	case    100: s2 = "500mS"; unit = 5000; break;
	case     50: s2 = "1S"; unit = 10000; break;
	case     25: s2 = "2S"; unit = 20000; break;
	case     10: s2 = "5S"; unit = 50000; break;
	}
	strcpy(b, s1);	
	strcat(b, s2);
	draw_text(308-(strlen(b)*12), 235, &hunter_12ptFontInfo, GY, &b[0]);
	int v = dy*yoffset;
	b[0] = v/100+'0';
	int off = 1;
	v = v%100;
	if (v) {
		b[off++] = '.';
		b[off++] = v/10+'0';
		v = v%10;
		if (v)
			b[off++] = v+'0';
	}
	b[off++] = 'V';
	b[off] = 0;
	draw_text(5, 15, &hunter_12ptFontInfo, GY, &b[0]);
	unit = unit*xoffset;
	if (unit >= 10000) {
		int v = unit/10000;
		unsigned char *cp = &b[0];
		unit = unit % 10000;
		cp += xnum(cp, v);
		if (unit) {
			*cp++ = '.';
			*cp++ = unit/1000+'0';
			unit = unit%1000;
			if (unit)
				*cp++ = unit/100+'0';
		}
		*cp++ = 'S';
		*cp = 0;
	} else
	if (unit >= 10) {
		int v = unit/10;
		unsigned char *cp = &b[0];
		unit = unit % 10;
		cp += xnum(cp, v);
		if (unit) {
			*cp++ = '.';
			*cp++ = unit+'0';
		}
		*cp++ = 'm';
		*cp++ = 'S';
		*cp = 0;
	} else 
	if (unit != 0) {
		unsigned char *cp = &b[0];
		*cp++ = unit+'0';
		*cp++ = '0';
		*cp++ = '0';
		*cp++ = 'u';
		*cp++ = 'S';
		*cp = 0;
	} else { // 0;
		unsigned char *cp = &b[0];
		*cp++ = '0';
		if (timebase >= 100000) {
			*cp++ = 'u';
		} else
		if (timebase >= 100) {
			*cp++ = 'm';
		} 
		*cp++ = 'S';
		*cp = 0;
	}
	draw_text(5, 235, &hunter_12ptFontInfo, GY, &b[0]);
	strcpy(b, "Pin");
	b[3] = '0'+scope_pin;
	b[4] = 0;
	draw_text(308-(strlen(b)*12), 15, &hunter_12ptFontInfo, GY, &b[0]);
	if (!running) {
		draw_trace(B);
	}
}

static
void scope_write()
{       
    uint16_t x,y;

    lcd_set_bounding_box(0, VS, 0, HS);
    
    spi_busy_wait(true);
    gpio_put(lcd_dp, 1);
    gpio_put(lcd_cs, 0);    
    
    for(uint32_t b=0; b<(HS*VS); b+=1){
        spi_write_blocking(BP_SPI_PORT, (unsigned char *)&clr[fb[b]], 2);
    }
    
    gpio_put(lcd_cs, 1);
    spi_busy_wait(false);
}



void
scope_lcd_update(uint32_t flags)
{
	if (!display)
		return;
	display = 0;
	draw_scope();
	spi_set_baudrate(BP_SPI_PORT, 62500*1000);
	scope_write();
	spi_set_baudrate(BP_SPI_PORT, 32000*1000);
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
