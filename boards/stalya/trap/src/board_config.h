/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * Board internal definitions
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <stm32_gpio.h>

/* PX4IO connection configuration */
#define BOARD_USES_PX4IO_VERSION       2
#define PX4IO_SERIAL_DEVICE            "/dev/ttyS4"
#define PX4IO_SERIAL_TX_GPIO           GPIO_USART6_TX
#define PX4IO_SERIAL_RX_GPIO           GPIO_USART6_RX
#define PX4IO_SERIAL_BASE              STM32_USART6_BASE
#define PX4IO_SERIAL_VECTOR            STM32_IRQ_USART6
#define PX4IO_SERIAL_TX_DMAMAP         DMAMAP_USART6_TX
#define PX4IO_SERIAL_RX_DMAMAP         DMAMAP_USART6_RX
#define PX4IO_SERIAL_RCC_REG           STM32_RCC_APB2ENR
#define PX4IO_SERIAL_RCC_EN            RCC_APB2ENR_USART6EN
#define PX4IO_SERIAL_CLOCK             STM32_PCLK2_FREQUENCY
#define PX4IO_SERIAL_BITRATE           1500000               /* 1.5Mbps -> max rate for IO */

/* LEDs */
#define GPIO_LED_RED       	/* PE13 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN13)
#define GPIO_LED_GREEN      	/* PE12 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN12)
#define GPIO_LED_BLUE       	/* PA5  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN5)

#define BOARD_HAS_CONTROL_STATUS_LEDS      1
#define BOARD_ARMED_STATE_LED  LED_RED

#define PIN_ADC3_INP1            	(GPIO_ANALOG|GPIO_PORTC|GPIO_PIN3)
#define PIN_ADC3_INP0            	(GPIO_ANALOG|GPIO_PORTC|GPIO_PIN2)
#define PIN_ADC3_INP10         		(GPIO_ANALOG|GPIO_PORTC|GPIO_PIN0)

/* ADC channels */
#define PX4_ADC_GPIO  \
	/* PC3 */  PIN_ADC3_INP1,	\
	/* PC2 */  PIN_ADC3_INP0,	\
	/* PC0 */  PIN_ADC3_INP10

/* Define Channel numbers must match above GPIO pins */
#define ADC_BATTERY_VOLTAGE_CHANNEL         1 	/* PC3: BATT_VOLTAGE_SENS */
#define ADC_BATTERY_CURRENT_CHANNEL         0 	/* PC2: BATT_CURRENT_SENS */
#define ADC_AIRSPEED_VOLTAGE_CHANNEL        10 	/* PC0: PRESSURE_SENS */

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL)  | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL)  | \
	 (1 << ADC_AIRSPEED_VOLTAGE_CHANNEL))

#define SYSTEM_ADC_BASE STM32_ADC3_BASE

/* HW has to large of R termination on ADC todo:change when HW value is chosen */
#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* CAN Silence Silent mode control */
#define GPIO_CAN1_SILENT_S0  	/* PD10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN10)

/* PWM */
#define DIRECT_PWM_OUTPUT_CHANNELS  4

/* Power supply control and monitoring GPIOs */
#define BOARD_NUMBER_BRICKS    	1
#define GPIO_VDD_USB_VALID      /* PA10 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTA|GPIO_PIN10)
#define GPIO_VDD_3V3_USB_EN     /* PE15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN15)
#define GPIO_VDD_3V3_GPS_EN     /* PA3  */ (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN3)
#define GPIO_VDD_4V1_GSM_EN     /* PC13 */ (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN13)
#define GPIO_VDD_3V3_EXT_EN     /* PA4  */ (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)

/* Tone alarm output */
#define TONE_ALARM_TIMER        3  /* timer 3 */
#define TONE_ALARM_CHANNEL      3  /* PB0 TIM3_CH3 */

#define GPIO_TONE_ALARM_IDLE    /* PB0 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN0) // ALARM
#define GPIO_TONE_ALARM         GPIO_TIM3_CH3OUT_1

/* USB
 *  OTG FS: PA10  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PA10 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_PORTA|GPIO_PIN10)

/* USB Hub */
#define GPIO_USBHUB_RESET     	/* PA9 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER               8  /* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL       3  /* use capture/compare channel 3 */

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_USB_VALID     BOARD_ADC_USB_CONNECTED
#define BOARD_ADC_BRICK_VALID  	(1)
#define BOARD_ADC_SERVO_VALID   (1)
#define BOARD_ADC_PERIPH_5V_OC  (0)
#define BOARD_ADC_HIPOWER_5V_OC (0)

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

#define BOARD_HAS_STATIC_MANIFEST 1

#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS
#define BOARD_NUM_IO_TIMERS 1

#define BOARD_DSHOT_MOTOR_ASSIGNMENT {3, 2, 1, 0};

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_I2C_BUS_CLOCK_INIT {400000}

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN1_SILENT_S0,		  \
		GPIO_USBHUB_RESET,		  \
		GPIO_VDD_3V3_USB_EN,          	  \
		GPIO_VDD_3V3_GPS_EN,           	  \
		GPIO_VDD_4V1_GSM_EN,           	  \
		GPIO_VDD_3V3_EXT_EN,          	  \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D0), \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_CMD),\
		GPIO_TONE_ALARM_IDLE,             \
		GPIO_OTGFS_VBUS,                  \
	}

__BEGIN_DECLS
#ifndef __ASSEMBLY__

extern void stm32_spiinitialize(void);
extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>
#endif /* __ASSEMBLY__ */
__END_DECLS
