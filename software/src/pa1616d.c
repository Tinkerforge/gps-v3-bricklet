/* gps-v3-bricklet
 * Copyright (C) 2022 Olaf Lüke <olaf@tinkerforge.com>
 *
 * pa1616d.c: Driver for PA1616D GPS module
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "pa1616d.h"

#include "configs/config_pa1616d.h"

#include "bricklib2/hal/system_timer/system_timer.h"
#include "bricklib2/utility/ringbuffer.h"
#include "bricklib2/utility/util_definitions.h"
#include "bricklib2/logging/logging.h"

#include "communication.h"

#include "xmc_uart.h"
#include "xmc_wdt.h"

#include <string.h>
#include <stdlib.h>

PA1616D pa1616d;

// Set const pointer to rx ringbuffer variables.
// With this the compiler can properly optimize the access!
uint8_t  *const pa1616d_ringbuffer_recv_buffer = &(pa1616d.buffer_recv[0]);
uint16_t *const pa1616d_ringbuffer_recv_end    = &(pa1616d.ringbuffer_recv.end);
uint16_t *const pa1616d_ringbuffer_recv_start  = &(pa1616d.ringbuffer_recv.start);
uint16_t *const pa1616d_ringbuffer_recv_size   = &(pa1616d.ringbuffer_recv.size);

#define pa1616d_recv_irq_handler  IRQ_Hdlr_11
void __attribute__((optimize("-O3"))) __attribute__ ((section (".ram_code"))) pa1616d_recv_irq_handler(void) {
	while(!XMC_USIC_CH_RXFIFO_IsEmpty(PA1616D_USIC)) {
		// Instead of ringbuffer_add() we add the byte to the buffer
		// by hand.
		//
		// We need to save the low watermark calculation overhead.

		uint16_t new_end = *pa1616d_ringbuffer_recv_end + 1;

		if(new_end >= *pa1616d_ringbuffer_recv_size) {
			new_end = 0;
		}

		if(new_end == *pa1616d_ringbuffer_recv_start) {
			// In the case of an overrun we read the byte and throw it away.
			volatile uint8_t __attribute__((unused)) _  = PA1616D_USIC->OUTR;
		} else {
			pa1616d_ringbuffer_recv_buffer[*pa1616d_ringbuffer_recv_end] = PA1616D_USIC->OUTR;
			*pa1616d_ringbuffer_recv_end = new_end;
		}
	}
}

void pa1616d_update_sbas(void) {
	if(pa1616d.sbas_enabled) {
		pa1616d.sbas = PA1616D_SBAS_ENABLE | PA1616D_SBAS_5HZ;
	} else {
		pa1616d.sbas = PA1616D_SBAS_DISABLE | PA1616D_SBAS_10HZ;
	}
}

void pa1616d_handle_sentence(const char *sentence) {
	enum minmea_sentence_id id = minmea_sentence_id(sentence, false);

	char talker_id[3];
	minmea_talker_id(talker_id, sentence);
	PA1616DTalker talker;

	if(talker_id[0] == 'G' && talker_id[1] == 'P') {
		talker = TALKER_GPS;
	} else if(talker_id[0] == 'G' && talker_id[1] == 'L') {
		talker = TALKER_GLONASS;
	} else if(talker_id[0] == 'G' && talker_id[1] == 'A') {
		talker = TALKER_GALILEO;
	} else if(talker_id[0] == 'G' && talker_id[1] == 'N') {
		talker = TALKER_MIXED;
	} else {
		// Unknown talker
		return;
	}

	pa1616d.last_data_time = system_timer_get_ms();


	switch(id) {
		case MINMEA_INVALID: {
			break;
		}

		case MINMEA_UNKNOWN: {
			break;
		}

		case MINMEA_SENTENCE_RMC: {
			struct minmea_sentence_rmc rmc;
			if(!minmea_parse_rmc(&rmc, sentence)) {
				return;
			}

			if(pa1616d.mixed.time.hours        != rmc.time.hours ||
			   pa1616d.mixed.time.minutes      != rmc.time.minutes ||
			   pa1616d.mixed.time.seconds      != rmc.time.seconds ||
			   pa1616d.mixed.time.microseconds != rmc.time.microseconds ||
			   pa1616d.mixed.date.year         != rmc.date.year ||
			   pa1616d.mixed.date.month        != rmc.date.month ||
			   pa1616d.mixed.date.day          != rmc.date.day) {
				pa1616d.new_date_time = true;
			}
			if(pa1616d.mixed.latitude.value    != rmc.latitude.value ||
			   pa1616d.mixed.longitude.value   != rmc.longitude.value) {
				pa1616d.new_coordinates = true;
			}
			if(pa1616d.mixed.speed.value       != rmc.speed.value ||
			   pa1616d.mixed.course.value      != rmc.course.value) {
				pa1616d.new_motion = true;
			}

			pa1616d.mixed.time      = rmc.time;
			pa1616d.mixed.valid     = rmc.valid;
			pa1616d.mixed.latitude  = rmc.latitude;
			pa1616d.mixed.longitude = rmc.longitude;
			pa1616d.mixed.speed     = rmc.speed; // knots
			pa1616d.mixed.course    = rmc.course;
			pa1616d.mixed.date      = rmc.date;
			pa1616d.mixed.variation = rmc.variation;

			break;
		}

		case MINMEA_SENTENCE_GGA: {
			struct minmea_sentence_gga gga;
			if(!minmea_parse_gga(&gga, sentence)) {
				return;
			}

			if(pa1616d.mixed.time.hours         != gga.time.hours ||
			   pa1616d.mixed.time.minutes       != gga.time.minutes ||
			   pa1616d.mixed.time.seconds       != gga.time.seconds ||
			   pa1616d.mixed.time.microseconds  != gga.time.microseconds) {
				pa1616d.new_date_time = true;
			}
			if(pa1616d.mixed.latitude.value     != gga.latitude.value ||
			   pa1616d.mixed.longitude.value    != gga.longitude.value) {
				pa1616d.new_coordinates = true;
			}
			if(pa1616d.mixed.fix_quality        != gga.fix_quality ||
			   pa1616d.mixed.satellites_tracked != gga.satellites_tracked) {
				pa1616d.new_status = true;
			}
			if(pa1616d.mixed.altitude.value     != gga.altitude.value ||
			   pa1616d.mixed.height.value       != gga.height.value) {
				pa1616d.new_altitude = true;
			}

			pa1616d.mixed.time               = gga.time;
			pa1616d.mixed.latitude           = gga.latitude;
			pa1616d.mixed.longitude          = gga.longitude;
			pa1616d.mixed.fix_quality        = gga.fix_quality;
			pa1616d.mixed.satellites_tracked = gga.satellites_tracked;
			pa1616d.mixed.hdop               = gga.hdop;
			pa1616d.mixed.altitude           = gga.altitude;
			pa1616d.mixed.altitude_units     = gga.altitude_units;
			pa1616d.mixed.height             = gga.height;
			pa1616d.mixed.height_units       = gga.height_units;
			pa1616d.mixed.dgps_age           = gga.dgps_age;

			break;
		}

		case MINMEA_SENTENCE_GSA: {
			struct minmea_sentence_gsa gsa;
			if(!minmea_parse_gsa(&gsa, sentence)) {
				return;
			}

			PA1616DDataSingle *single;
			if(talker == TALKER_GPS) {
				single = &pa1616d.gps;
			} else if(talker == TALKER_GLONASS) {
				single = &pa1616d.glonass;
			} else if(talker == TALKER_GALILEO) {
				single = &pa1616d.galileo;
			} else {
				return;
			}

		    single->mode = gsa.mode;
		    single->fix_type = gsa.fix_type;
		    memcpy(single->sats, gsa.sats, 12);
		    single->pdop = gsa.pdop;
		    single->hdop = gsa.hdop;
		    single->vdop = gsa.vdop;

			break;
		}

		case MINMEA_SENTENCE_GLL: {
			// Not supported by PA1616D
			break;
		}

		case MINMEA_SENTENCE_GST: {
			// Not supported PA1616D
			break;
		}

		case MINMEA_SENTENCE_GSV: {
			struct minmea_sentence_gsv gsv;
			if(!minmea_parse_gsv(&gsv, sentence)) {
				return;
			}

			PA1616DDataSingle *single;
			if(talker == TALKER_GPS) {
				single = &pa1616d.gps;
			} else if(talker == TALKER_GLONASS) {
				single = &pa1616d.glonass;
			} else if(talker == TALKER_GALILEO) {
				single = &pa1616d.galileo;
			} else {
				return;
			}

			for(uint8_t i = 0; i < 4; i++) {
				uint16_t pnr = gsv.sats[i].nr;
				if(pnr == 96) { // pnr 96 seems to always produce garbage values, we ignore it
					continue;
				}

				if(talker == TALKER_GLONASS) {
					pnr -= 64; // GLONASS IDs go from 65 to 96
				} else if(talker == TALKER_GALILEO) {
					//pnr -= 300; // GALILEO IDs go from 301 to 336 but is given as 1-36 by PD1616D
				}
				if(pnr < 1 || pnr > 32) { // pnr should always be between 1 and 32
					continue;
				}

				single->sat_info[pnr-1].nr        = gsv.sats[i].nr;
				single->sat_info[pnr-1].azimuth   = gsv.sats[i].azimuth;
				single->sat_info[pnr-1].elevation = gsv.sats[i].elevation;
				single->sat_info[pnr-1].snr       = gsv.sats[i].snr;
			}

			break;
		}

		case MINMEA_SENTENCE_VTG: {
			struct minmea_sentence_vtg vtg;
			if(!minmea_parse_vtg(&vtg, sentence)) {
				return;
			}

			pa1616d.mixed.true_track_degrees     = vtg.true_track_degrees;
			pa1616d.mixed.magnetic_track_degrees = vtg.magnetic_track_degrees;
			pa1616d.mixed.speed_knots            = vtg.speed_knots;
			pa1616d.mixed.speed_kph              = vtg.speed_kph;
			pa1616d.mixed.faa_mode               = vtg.faa_mode;

			break;
		}
	}
}

void pa1616d_handle_send(void) {
	if(pa1616d.send_index == -1) {
		if(pa1616d.restart != 0) {
			memset(&pa1616d.mixed, 0, sizeof(PA1616DDataMixed));
			memset(&pa1616d.gps, 0, sizeof(PA1616DDataSingle));
			memset(&pa1616d.glonass, 0, sizeof(PA1616DDataSingle));
		}

		if(system_timer_is_time_elapsed_ms(pa1616d.last_send_time, PA1616D_TIME_BETWEEN_SENDS)) {
			if(pa1616d.restart & PA1616D_RESTART_HOT) {
				strcpy(pa1616d.buffer_send, "$PMTK101*32\r\n");
				pa1616d.restart &= ~PA1616D_RESTART_HOT;
			} else if(pa1616d.restart & PA1616D_RESTART_WARM) {
				strcpy(pa1616d.buffer_send, "$PMTK102*31\r\n");
				pa1616d.restart &= ~PA1616D_RESTART_WARM;
			} else if(pa1616d.restart & PA1616D_RESTART_COLD) {
				strcpy(pa1616d.buffer_send, "$PMTK103*30\r\n");
				pa1616d.restart &= ~PA1616D_RESTART_COLD;
				pa1616d_update_sbas(); // cold restart clears SBAS settings
			} else if(pa1616d.restart & PA1616D_RESTART_FACTORY) {
				strcpy(pa1616d.buffer_send, "$PMTK104*37\r\n");
				pa1616d.restart &= ~PA1616D_RESTART_FACTORY;
				pa1616d_update_sbas(); // factory-reset clears SBAS settings
			} else if(pa1616d.sbas & PA1616D_SBAS_BAUDRATE) {
				strcpy(pa1616d.buffer_send, "$PMTK251,115200*1F\r\n");
				pa1616d.sbas &= ~PA1616D_SBAS_BAUDRATE;
			} else if(pa1616d.sbas & PA1616D_SBAS_DISABLE) {
				strcpy(pa1616d.buffer_send, "$PMTK313,0*2F\r\n");
				pa1616d.sbas &= ~PA1616D_SBAS_DISABLE;
			} else if(pa1616d.sbas & PA1616D_SBAS_10HZ) {
				strcpy(pa1616d.buffer_send, "$PMTK220,100*2F\r\n");
				pa1616d.sbas &= ~PA1616D_SBAS_10HZ;
			} else if(pa1616d.sbas & PA1616D_SBAS_5HZ) {
				strcpy(pa1616d.buffer_send, "$PMTK220,200*2C\r\n");
				pa1616d.sbas &= ~PA1616D_SBAS_5HZ;
			} else if(pa1616d.sbas & PA1616D_SBAS_ENABLE) {
				strcpy(pa1616d.buffer_send, "$PMTK313,1*2E\r\n");
				pa1616d.sbas &= ~PA1616D_SBAS_ENABLE;
			}

			if(pa1616d.buffer_send[0] == '$') {
				pa1616d.send_index     = 0;
				pa1616d.last_send_time = system_timer_get_ms();
			}
		}
	}

	const uint8_t length = strlen(pa1616d.buffer_send);
	if(length != 0) {
		while(pa1616d.send_index < length) {
			if(!XMC_USIC_CH_TXFIFO_IsFull(PA1616D_USIC)) {
				PA1616D_USIC->IN[0] = pa1616d.buffer_send[pa1616d.send_index];
				pa1616d.send_index++;
			}
		}
		if(pa1616d.send_index >= length) {
			memset(pa1616d.buffer_send, 0, PA1616D_SEND_BUFFER_SIZE);
			pa1616d.send_index = -1;
		}
	}
}

void pa1616d_handle_recv(void) {
	static char sentence[PA1616D_MAX_SENTENCE_LENGTH+1] = {0};
	static int16_t index = 0;

	while(index < PA1616D_MAX_SENTENCE_LENGTH) {
		// Turn RX interrupt off during ringbuffer_get
		NVIC_DisableIRQ((IRQn_Type)PA1616D_IRQ_RX);
		__DSB();
		__ISB();
		if(!ringbuffer_get(&pa1616d.ringbuffer_recv, (uint8_t*)&sentence[index])) {
			NVIC_EnableIRQ((IRQn_Type)PA1616D_IRQ_RX);
			break;
		}
		NVIC_EnableIRQ((IRQn_Type)PA1616D_IRQ_RX);

		if(sentence[index] == '\n') {
			continue;
		}
		if(sentence[index] == '\r') {
			sentence[index] = '\0';

			pa1616d_handle_sentence(sentence);
			memset(sentence, 0 , index+1);

			index = 0;

			// We return after each sentence, so we can return to bootloader
			// between sentences
			break;
		}
		index++;
	}

	if(index == PA1616D_MAX_SENTENCE_LENGTH) {
		// Sentence was longer then PA1616D_MAX_SENTENCE_LENGTH byte, we don't support this
		memset(sentence, 0 , PA1616D_MAX_SENTENCE_LENGTH);
		index = 0;
	}
}

void pa1616d_init_hardware(void) {
	// Interrupt pin configuration
	const XMC_GPIO_CONFIG_t fix_pin_config = {
		.mode             = XMC_GPIO_MODE_INPUT_TRISTATE,
		.output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH,
		.input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD
	};

	// Reset pin configuration
	const XMC_GPIO_CONFIG_t nreset_pin_config = {
		.mode             = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
		.output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH, // not reset
	};

	// Antenna pin configuration
	const XMC_GPIO_CONFIG_t antenna_pin_config = {
		.mode             = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
		.output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH, // internal
	};

	// PPS pin configuration
	const XMC_GPIO_CONFIG_t pps_pin_config = {
		.mode             = XMC_GPIO_MODE_INPUT_TRISTATE,
		.output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH,
		.input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD
	};

	// LED pin configuration
	XMC_GPIO_CONFIG_t led_pin_config = {
		.mode             = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
		.output_level     = XMC_GPIO_OUTPUT_LEVEL_LOW
	};


	// Configure GPIO pins
	XMC_GPIO_Init(PA1616D_3DFIX_PIN, &fix_pin_config);
	XMC_GPIO_Init(PA1616D_NRESET_PIN, &nreset_pin_config);
	XMC_GPIO_Init(PA1616D_ANT_SWITCH_PIN, &antenna_pin_config);
	XMC_GPIO_Init(PA1616D_PPS_PIN, &pps_pin_config);
	XMC_GPIO_Init(PA1616D_FIX_LED_PIN, &led_pin_config);


	// TX pin configuration
	const XMC_GPIO_CONFIG_t tx_pin_config = {
		.mode             = PA1616D_TX_PIN_AF,
		.output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH
	};

	// RX pin configuration
	const XMC_GPIO_CONFIG_t rx_pin_config = {
		.mode             = XMC_GPIO_MODE_INPUT_PULL_UP,
		.input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD
	};

	// Configure pins
	XMC_GPIO_Init(PA1616D_TX_PIN, &tx_pin_config);
	XMC_GPIO_Init(PA1616D_RX_PIN, &rx_pin_config);

	// Initialize USIC channel in UART master mode
	// USIC channel configuration
	XMC_UART_CH_CONFIG_t config;
	config.oversampling = 16;
	config.frame_length = 8;
	config.baudrate     = 9600;
	config.stop_bits    = 1;
	config.data_bits    = 8;
	config.parity_mode  = XMC_USIC_CH_PARITY_MODE_NONE;
	XMC_UART_CH_Init(PA1616D_USIC, &config);

	// Set input source path
	XMC_UART_CH_SetInputSource(PA1616D_USIC, PA1616D_RX_INPUT, PA1616D_RX_SOURCE);

	// Configure transmit FIFO
	XMC_USIC_CH_TXFIFO_Configure(PA1616D_USIC, 32, XMC_USIC_CH_FIFO_SIZE_16WORDS, 8);

	// Configure receive FIFO
	XMC_USIC_CH_RXFIFO_Configure(PA1616D_USIC, 48, XMC_USIC_CH_FIFO_SIZE_16WORDS, 0);

	// Set service request for tx FIFO transmit interrupt
	XMC_USIC_CH_TXFIFO_SetInterruptNodePointer(PA1616D_USIC, XMC_USIC_CH_TXFIFO_INTERRUPT_NODE_POINTER_STANDARD, PA1616D_SERVICE_REQUEST_TX);

	// Set service request for rx FIFO receive interrupt
	XMC_USIC_CH_RXFIFO_SetInterruptNodePointer(PA1616D_USIC, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_STANDARD, PA1616D_SERVICE_REQUEST_RX);
	XMC_USIC_CH_RXFIFO_SetInterruptNodePointer(PA1616D_USIC, XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_ALTERNATE, PA1616D_SERVICE_REQUEST_RX);

	// Set priority and enable NVIC node for receive interrupt
	NVIC_SetPriority(PA1616D_IRQ_RX, PA1616D_IRQ_RX_PRIORITY);
	NVIC_EnableIRQ(PA1616D_IRQ_RX);

	// Start UART
	XMC_UART_CH_Start(PA1616D_USIC);

	XMC_USIC_CH_EnableEvent(PA1616D_USIC, XMC_USIC_CH_EVENT_ALTERNATIVE_RECEIVE);
	XMC_USIC_CH_RXFIFO_EnableEvent(PA1616D_USIC, XMC_USIC_CH_RXFIFO_EVENT_CONF_STANDARD | XMC_USIC_CH_RXFIFO_EVENT_CONF_ALTERNATE);
}

void pa1616d_init_buffer(void) {
	pa1616d.wait_8ms_start_time = 0;
	pa1616d.fix_led_state.config = LED_FLICKER_CONFIG_EXTERNAL;
	pa1616d.fix_led_config = GPS_V3_FIX_LED_CONFIG_SHOW_FIX;

	// Disable interrupts so we can't accidentally
	// receive ringbuffer_adds in between a re-init
	NVIC_DisableIRQ(PA1616D_IRQ_RX);
	__DSB();
	__ISB();

	// Initialize pa1616d buffer
	memset(pa1616d.buffer_recv, 0, PA1616D_RECV_BUFFER_SIZE);
	memset(pa1616d.buffer_send, 0, PA1616D_SEND_BUFFER_SIZE);
	pa1616d.send_index = -1;

	ringbuffer_init(&pa1616d.ringbuffer_recv, PA1616D_RECV_BUFFER_SIZE, pa1616d.buffer_recv);

	NVIC_EnableIRQ(PA1616D_IRQ_RX);
}


void pa1616d_handle_fix_led(void) {
	static bool last_pps_high = false;
	static uint32_t last_pps_led_on = 0;

	// Handle standard configurations (on, off, heartbeat)
	led_flicker_tick(&pa1616d.fix_led_state, system_timer_get_ms(), PA1616D_FIX_LED_PIN);

	// Handle fix configuration
	if(pa1616d.fix_led_config == GPS_V3_FIX_LED_CONFIG_SHOW_FIX) {
		if(pa1616d.mixed.fix_quality == 0) {
			XMC_GPIO_SetOutputHigh(PA1616D_FIX_LED_PIN);
		} else {
			XMC_GPIO_SetOutputLow(PA1616D_FIX_LED_PIN);
		}
	} else if(pa1616d.fix_led_config == GPS_V3_FIX_LED_CONFIG_SHOW_PPS) {
		if(XMC_GPIO_GetInput(PA1616D_PPS_PIN)) {
			if(!last_pps_high) {
				XMC_GPIO_SetOutputLow(PA1616D_FIX_LED_PIN);
				last_pps_led_on = system_timer_get_ms();
				last_pps_high = true;
			}
		} else {
			last_pps_high = false;
		}

		if(system_timer_is_time_elapsed_ms(last_pps_led_on, PA1616D_PPS_BLINK_TIME)) {
			XMC_GPIO_SetOutputHigh(PA1616D_FIX_LED_PIN);
		}
	}
}

void pa1616d_tick_init(void) {
	pa1616d.last_data_time = system_timer_get_ms();
	pa1616d.last_interrupt_time = system_timer_get_ms();
	pa1616d.last_send_time = system_timer_get_ms();

	pa1616d.sbas = PA1616D_SBAS_BAUDRATE;
	// Busy-wait until PA1616D baudrate is changed from 9600 to 115200
	do {
		pa1616d_handle_send();
	} while((pa1616d.send_index != -1) || (pa1616d.sbas == PA1616D_SBAS_BAUDRATE));

	// Wait for baudrate change command to be completely send
	while(XMC_USIC_CH_TXFIFO_GetLevel(PA1616D_USIC) != 0) {
		__NOP();
	}
	system_timer_sleep_ms(10); // Wait a bit more to be sure that the last byte is out of the shift register

	// Change baudrate of USIC hardware unit to 115200
	XMC_USIC_CH_SetBaudrate(PA1616D_USIC, 115200, 16);

	pa1616d.sbas_enabled = true;
	pa1616d_update_sbas();
}

void pa1616d_tick(void) {
	// Wait for 1s for PA1616D to boot before we try to initialize it
	if((pa1616d.tick_init_time != 0) && (system_timer_is_time_elapsed_ms(pa1616d.tick_init_time, 1000))) {
		pa1616d_tick_init();
		pa1616d.tick_init_time = 0;
	} else {
		pa1616d_handle_recv();
		pa1616d_handle_send();
		pa1616d_handle_fix_led();
	}
}

void pa1616d_init(void) {
	system_timer_sleep_ms(500);
	memset(&pa1616d, 0, sizeof(PA1616D));

	pa1616d_init_buffer();
	pa1616d_init_hardware();

	pa1616d.tick_init_time = system_timer_get_ms();
}