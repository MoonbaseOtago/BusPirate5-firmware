
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

#define CAPTURE_DEPTH 64
#define BUFFERS 32
static uint16_t buffer[BUFFERS*CAPTURE_DEPTH];
static int offset =0;
static uint dma_chan;
static unsigned char data_ready;
static unsigned char stop_capture;
static int int_count=0;
static uint16_t last_value, trigger_level=1<<10;
static uint32_t trigger_offset;		// offset from start of dsplay in points
static uint32_t trigger_point;		// index of trigger point
static unsigned char search_rising, search_falling, search_either, first_sample;
static uint32_t timebase = 500000; // fastest possible
static uint32_t h_res = 100;	// 100uS
static uint8_t scope_pin = 2;
static unsigned char fb[240*320];
typedef enum { SMODE_ONCE, SMODE_NORMAL, SMODE_AUTO } SCOPE_MODE;
SCOPE_MODE scope_mode=SMODE_NORMAL;
typedef enum { TRIGGER_POS, TRIGGER_NEG, TRIGGER_NONE, TRIGGER_BOTH } TRIGGER_TYPE;
TRIGGER_TYPE trigger_type = TRIGGER_BOTH;

unsigned short clr[] = 
{
	0x0000,	// black
	0xffff,	// white
	0x07ff, // yellow
	0x187f, // blue
};

typedef enum {
BL=0,
WH=1,
Y=2,
B=3,
} CLR;




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
		printf("	0-7 - input pin\r\n");
		printf("	o - once\r\n");
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
	adc_set_clkdiv(0);
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
for (int i = 0; i < 10; i++) printf("%04x ", buffer[i]);printf("\r\n");

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
	printf("up\r\n");
}

static void
scope_down(void)
{
	printf("down\r\n");
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
//	sx - scope X  up/down (scale) left right
//	sy - scope y
//  st - scope trigger +-nb up down left right ports- num 
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
		while ((c = next_char()) != 0) {
			switch (c) {
			case 0x1b:
				c = next_char();
				if (c != '[')
					goto bade;
				c = next_char();
				switch (c) {
				case 'A': trigger_up();	break;
				case 'B': trigger_down();	break;
				case 'C': trigger_right();break;
				case 'D': trigger_left();	break;
				default:
bade:
					printf("bad escape sequence\r\n");
					return 0;
				}
				break;
			case '+':	trigger_type = TRIGGER_POS; break;
			case '-':	trigger_type = TRIGGER_NEG; break;
			case 'n':	trigger_type = TRIGGER_NONE; break;
			case 'b':	trigger_type = TRIGGER_BOTH; break;
			case 'v':
			case 'V':	trigger_down(); break;
			case '^':	trigger_up(); break;
			case '<':	trigger_left(); break;
			case '>':	trigger_right(); break;
			default:	
				printf("unknown command\r\n");
				return 0;
			}
		}
	} else
	if (strcmp(args[0].c, "sx") == 0)  {
		// x - timebase
		// < move left
		// > move right
		// + faster
		// - slower
		while ((c = next_char()) != 0) {
			switch (c) {
			case 0x1b:
				c = next_char();
				if (c != '[')
					goto bade;
				c = next_char();
				switch (c) {
				case 'A': scope_up();	break;
				case 'B': scope_down();	break;
				case 'C': scope_right();break;
				case 'D': scope_left();	break;
				default:
					printf("bad escape sequence\r\n");
					return 0;
				}
				break;
			case '+':	;
			case '-':	;
			case 'v':
			case 'V':	scope_down(); break;
			case '^':	scope_up(); break;
			case '<':	scope_left(); break;
			case '>':	scope_right(); break;
			default:	
				printf("unknown command\r\n");
				return 0;
			}
		}
	} else
	if (strcmp(args[0].c, "sy") == 0)  {
		// y - y scale
		// + increase scale
		// - decrease scale
		// ^ move up
		// v move down
		while ((c = next_char()) != 0) {
			switch (c) {
			case 0x1b:
				c = next_char();
				if (c != '[')
					goto bade;
				c = next_char();
				switch (c) {
				case 'A': scope_up();	break;
				case 'B': scope_down();	break;
				case 'C': scope_right();break;
				case 'D': scope_left();	break;
				default:
					printf("bad escape sequence\r\n");
					return 0;
				}
				break;
			case '+':	;
			case '-':	;
			case 'v':
			case 'V':	scope_down(); break;
			case '^':	scope_up(); break;
			case '<':	scope_left(); break;
			case '>':	scope_right(); break;
			default:	
				printf("unknown command\r\n");
				return 0;
			}
		}
	} else
	if (strcmp(args[0].c, "sr") == 0)  {
		// start 
		// <> same as last - button if stopped
		// 0-7 - pin
		// o - once
		// n - normal
		// a - auto
		for (int i = 0; i < 5;  i++) {
			opt_args r;
			if (!ui_parse_get_string(&r))
				break;
			for (char *cp=&r.c[0]; *cp; cp++)
			if (*cp >= '0' && *cp < '7') {
				scope_pin = *cp-'0';
			} else
			if (*cp == 'o' || *cp == 'O') {
				scope_mode = SMODE_ONCE;
			} else 
			if (*cp == 'n' || *cp == 'N') {
				scope_mode = SMODE_NORMAL;
			} else 
			if (*cp == 'a' || *cp == 'A') {
				scope_mode = SMODE_AUTO;
			} else  {
				printf("in valid start mode '%s'\r\n", r.c);
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
	unsigned char *p = &fb[y1*320+x1];
	for (int i = x1; i <= x2; i++)
		*p++ = c;
}
static void
draw_v_line(int x1, int y1, int y2, CLR c)
{
	unsigned char *p = &fb[y1*320+x1];
	for (int i = y1; i <= y2; i++) {
		*p = c;
		p += 320;
	}
}

static void
draw_grid(CLR c)
{
	scope_fb_init();
	for (int i = 0; i < 5 ; i++) 
		draw_h_line(0, 319, i*(240/5), c);
	draw_h_line(0, 319, 239, c);
	for (int i = 0; i < 7 ; i++) 
		draw_v_line(i*50, 0, 239, c);
	//draw_v_line(319, 0, 239, c);
}

static void 
draw_text(int x, int y, const FONT_INFO *font, CLR c, char *cp) // stolen from lcd_write_string
{
	unsigned char *b = &fb[x];
	int y320 = 320*y;
	while (*cp) {
		uint8_t adjusted_c = (*cp)-(*font).start_char;
		for (uint16_t col=0; col < (*font).lookup[adjusted_c].width; col++) {
			uint16_t row=0;
			uint16_t rows = (*font).lookup[adjusted_c].height;
			uint16_t offset=(*font).lookup[adjusted_c].offset;
			int off = y320;
			for (uint16_t page=0; page<(*font).height_bytes; page++) {
				uint8_t bitmap_char = (*font).bitmaps[offset+(col*(*font).height_bytes)+page];
				for (uint8_t i=0; i<8; i++) {
					if (off >= 0 && bitmap_char&(0x80>>i)) {
						b[off] = c;
					} 
					off -= 320;
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
	uint32_t v[320];
	for (int x = 0; x < 320; x++) {
		uint32_t y = *b++;
		
		v[x] = y*240/V5;
	}
	unsigned y1 = v[0];
	unsigned y2 = (v[1]+v[0])>>1;
	if (y1 < 240 || y2 < 240) {
		if (y1 >= 240) y1 = 239;
		if (y2 >= 240) y2 = 239;
		if (y1 > y2) {
			int t = y1;
			y1 = y2;
			y2 = t;
		}
		for (int y = y1; y <= y2; y++)
			p[320*y+0] = c;
	}
	for (int x=1; x < 319; x++) {
		unsigned y1 = (v[x-1]+v[x])>>1;
		unsigned y2 = (v[x+1]+v[x])>>1;
		if (y1 >= 240) {
			if (y2 >= 240)
				continue;
			y1 = 239;
		} else
		if (y2 >= 240) {
			y2 = 239;
		}
		if (y1 > y2) {
			int t = y1;
			y1 = y2;
			y2 = t;
		}
		for (int y = y1; y <= y2; y++)
			p[320*y+x] = c;
	}
	y1 = v[319];
	y2 = (v[318]+v[319])>>1;
	if (y1 < 240 || y2 < 240) {
		if (y1 >= 240) y1 = 239;
		if (y2 >= 240) y2 = 239;
		if (y1 > y2) {
			int t = y1;
			y1 = y2;
			y2 = t;
		}
		for (int y = y1; y <= y2; y++)
			p[320*y+319] = c;
	}
}


static void 
draw_scope()
{
	draw_grid(Y);
	draw_text(10, 20, &hunter_12ptFontInfo, WH, "Test");
	draw_trace(B);
}

static
void scope_write()
{       
    uint16_t x,y;

    lcd_set_bounding_box(0, 240, 0, 320);
    
    spi_busy_wait(true);
    gpio_put(lcd_dp, 1);
    gpio_put(lcd_cs, 0);    
    
    for(uint32_t b=0; b<(320*240); b+=1){
        spi_write_blocking(BP_SPI_PORT, (unsigned char *)&clr[fb[b]], 2);
    }
    
    gpio_put(lcd_cs, 1);
    spi_busy_wait(false);
}



void
scope_lcd_update(uint32_t flags)
{
static int i;
	draw_scope();
spi_set_baudrate(BP_SPI_PORT, 1000*1000*72);
	scope_write();
spi_set_baudrate(BP_SPI_PORT, 1000*1000*32);
i++;
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
