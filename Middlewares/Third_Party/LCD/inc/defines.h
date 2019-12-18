/*
 * defines.h
 *
 *  Created on: 16 de dez de 2019
 *      Author: Fernando
 */

#ifndef THIRD_PARTY_LCD_INC_DEFINES_H_
#define THIRD_PARTY_LCD_INC_DEFINES_H_


/*
 * Display Macros for Pin definition
4 - DB4 - PB5
5 - DB5 - PB4
6 - DB6 - PB10
7 - DB7 - PA8
8 - RS  - PA9
9 - EN	- PC7
 *
 */
//RS - Register select pin
#define HD44780_RS_PORT     GPIOA
#define HD44780_RS_PIN      GPIO_PIN_9
//E - Enable pin
#define HD44780_E_PORT      GPIOC
#define HD44780_E_PIN       GPIO_PIN_7
//D4 - Data 4 pin
#define HD44780_D4_PORT     GPIOB
#define HD44780_D4_PIN      GPIO_PIN_5
//D5 - Data 5 pin
#define HD44780_D5_PORT     GPIOB
#define HD44780_D5_PIN      GPIO_PIN_4
//D6 - Data 6 pin
#define HD44780_D6_PORT     GPIOB
#define HD44780_D6_PIN      GPIO_PIN_10
//D7 - Data 7 pin
#define HD44780_D7_PORT     GPIOA
#define HD44780_D7_PIN      GPIO_PIN_8



#endif /* THIRD_PARTY_LCD_INC_DEFINES_H_ */
