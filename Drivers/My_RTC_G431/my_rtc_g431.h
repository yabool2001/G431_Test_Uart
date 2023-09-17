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

void set_rtc_time ( uint8_t Hours , uint8_t Minutes , uint32_t Seconds , uint8_t WeekDay , uint8_t Month , uint8_t Date , uint8_t Year ) ;
void get_rtc_time ( void ) ;
void set_rtc_alarm (uint8_t Hours , uint8_t Minutes , uint32_t Seconds , uint8_t Date ) ;

#endif /* MY_RTC_G431_MY_RTC_G431_H_ */
