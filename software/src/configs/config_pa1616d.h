/* o3-bricklet
 * Copyright (C) 2019 Olaf Lüke <olaf@tinkerforge.com>
 *
 * config_pa1616d.h: Config for DGS-O3 sensor
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

#ifndef CONFIG_PA1616D_H
#define CONFIG_PA1616D_H

#include "xmc_common.h"
#include "xmc_gpio.h"

#define PA1616D_USIC_CHANNEL        USIC0_CH1
#define PA1616D_USIC                XMC_UART0_CH1

#define PA1616D_RX_PIN              P2_11
#define PA1616D_RX_INPUT            XMC_USIC_CH_INPUT_DX0
#define PA1616D_RX_SOURCE           0b100 // DX0E

#define PA1616D_TX_PIN              P2_10
#define PA1616D_TX_PIN_AF           (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7 | P2_10_AF_U0C1_DOUT0)

#define PA1616D_SERVICE_REQUEST_RX  2  // receive
#define PA1616D_SERVICE_REQUEST_TX  3  // transfer

#define PA1616D_IRQ_RX              11
#define PA1616D_IRQ_RX_PRIORITY     0

#define PA1616D_FIX_LED_PIN         P1_1
#define PA1616D_3DFIX_PIN           P1_0
#define PA1616D_NRESET_PIN          P0_0
#define PA1616D_PPS_PIN             P0_5
#define PA1616D_ANT_SWITCH_PIN      P0_6

#endif