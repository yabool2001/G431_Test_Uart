/*
 * my_rtc_g431.h
 *
 *  Created on: Sep 17, 2023
 *      Author: mzeml
 */

#include "main.h"
#include <stdio.h>

#ifndef MY_RTC_G431_MY_RTC_G431_H_
#define MY_RTC_G431_MY_RTC_G431_H_

void set_rtc_time ( uint8_t , uint8_t , uint32_t , uint8_t , uint8_t , uint8_t , uint8_t ) ;
void get_rtc_time ( void ) ;
void set_rtc_alarm (uint8_t , uint8_t , uint32_t , uint8_t ) ;
void send_debug_logs ( char* ) ;

#endif /* MY_RTC_G431_MY_RTC_G431_H_ */
