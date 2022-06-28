/* gps-v3-bricklet
 * Copyright (C) 2022 Olaf Lüke <olaf@tinkerforge.com>
 *
 * pa1616d.h: Driver for PA1616D GPS module
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

#ifndef PA1616D_H
#define PA1616D_H

#include <stdint.h>

#include "bricklib2/utility/ringbuffer.h"
#include "bricklib2/utility/led_flicker.h"

#include "configs/config_pa1616d.h"

#include "minmea.h"

#define PA1616D_PPS_BLINK_TIME 200 // Blink duration in ms
#define PA1616D_MAX_SENTENCE_LENGTH 512

#define PA1616D_RECV_BUFFER_SIZE 1024
#define PA1616D_SEND_BUFFER_SIZE 128

#define PA1616D_RECV_BUFFER_MASK (PA1616D_RECV_BUFFER_SIZE-1) // Use power of two size only here!

#define PA1616D_MAX_SAT_NUM 32

#define PA1616D_INTERRUPT_TIMEOUT 2000
#define PA1616D_DATA_TIMEOUT 10000
#define PA1616D_RESET_TIMEOUT 1000
#define PA1616D_TIME_BETWEEN_SENDS 100 // According to datasheet we need to wait 30ms between each message that we send


typedef struct {
    char mode;
    int fix_type;
    int sats[12];
    struct minmea_float pdop;
    struct minmea_float hdop;
    struct minmea_float vdop;

    struct minmea_sat_info sat_info[PA1616D_MAX_SAT_NUM];
} PA1616DDataSingle;

typedef struct {
    struct minmea_time time;
    bool valid;
    struct minmea_float latitude;
    struct minmea_float longitude;
    struct minmea_float speed; // knots
    struct minmea_float course;
    struct minmea_date date;
    struct minmea_float variation;
    int fix_quality;
    int satellites_tracked;
    struct minmea_float hdop;
    struct minmea_float altitude; char altitude_units;
    struct minmea_float height; char height_units;
    int dgps_age;
    struct minmea_float true_track_degrees;
    struct minmea_float magnetic_track_degrees;
    struct minmea_float speed_knots;
    struct minmea_float speed_kph;
    enum minmea_faa_mode faa_mode;
} PA1616DDataMixed;

typedef enum {
	PA1616D_RESTART_HOT     = 1,
	PA1616D_RESTART_WARM    = 2,
	PA1616D_RESTART_COLD    = 4,
	PA1616D_RESTART_FACTORY = 8,
} PA1616DRestart;

typedef enum {
	PA1616D_SBAS_5HZ      = 1,
	PA1616D_SBAS_ENABLE   = 2,
	PA1616D_SBAS_DISABLE  = 4,
	PA1616D_SBAS_10HZ     = 8,
	PA1616D_SBAS_BAUDRATE = 16,
} PA1616DSBAS;

typedef enum {
	PA1616D_STATE_WAIT_FOR_INTERRUPT,
	PA1616D_STATE_RECEIVE_IN_PROGRESS,
	PA1616D_STATE_WAIT_8MS,
	PA1616D_STATE_RESET,
} PA1616DState;

typedef enum {
	TALKER_MIXED,
	TALKER_GPS,
	TALKER_GLONASS,
	TALKER_GALILEO,
} PA1616DTalker;

typedef struct {
	uint8_t buffer_recv[PA1616D_RECV_BUFFER_SIZE];
	char buffer_send[PA1616D_SEND_BUFFER_SIZE];
	Ringbuffer ringbuffer_recv;

	uint16_t buffer_recv_counter;
	PA1616DState state;
	uint32_t wait_8ms_start_time;
	LEDFlickerState fix_led_state;
	uint8_t fix_led_config;

	uint32_t last_interrupt_time;
	uint32_t last_data_time;
	uint32_t reset_time;
	uint32_t last_send_time;
	uint32_t tick_init_time;
    int16_t send_index;

	uint8_t restart;
	uint8_t sbas;
	bool sbas_enabled;

	bool new_coordinates;
	bool new_status;
	bool new_altitude;
	bool new_motion;
	bool new_date_time;

	PA1616DDataMixed mixed;
	PA1616DDataSingle gps;
	PA1616DDataSingle glonass;
	PA1616DDataSingle galileo;

} PA1616D;

extern PA1616D pa1616d;

void pa1616d_tick(void);
void pa1616d_init(void);
void pa1616d_update_sbas(void);

#endif