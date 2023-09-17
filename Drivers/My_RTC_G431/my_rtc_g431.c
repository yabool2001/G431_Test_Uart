/*
 * my_rtc_g431.c
 *
 *  Created on: Sep 17, 2023
 *      Author: mzeml
 */

#include "my_rtc_g431.h"

extern RTC_HandleTypeDef hrtc ;

void set_rtc_time ( uint8_t Hours , uint8_t Minutes , uint32_t Seconds , uint8_t WeekDay , uint8_t Month , uint8_t Date , uint8_t Year )
{
	RTC_TimeTypeDef sTime ;
	RTC_DateTypeDef sDate ;

	sTime.Hours = Hours ;
	sTime.Minutes = Minutes ;
	sTime.Seconds = Seconds ;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET ;

	if ( HAL_RTC_SetTime ( &hrtc , &sTime , RTC_FORMAT_BCD ) != HAL_OK )
	{
	}

	sDate.WeekDay = WeekDay ;
	sDate.Month = Month ;
	sDate.Date = Date ;
	sDate.Year = Year ;

	if ( HAL_RTC_SetDate ( &hrtc , &sDate , RTC_FORMAT_BCD ) != HAL_OK )
	{
	}

  // Call HAL_RTCEx_BKUPWrite with parameters hrtc, RTC_BKP_DR1 and dumy value like 0x32F2
	HAL_RTCEx_BKUPWrite ( &hrtc , RTC_BKP_DR1 , 0x32F2 ) ;
}

void get_rtc_time ( void )
{
	RTC_DateTypeDef gDate ;
	RTC_TimeTypeDef gTime ;

	char ctime[9] ;
	char cdate[11] ;

	HAL_RTC_GetTime ( &hrtc , &gTime , RTC_FORMAT_BIN ) ;
	HAL_RTC_GetDate ( &hrtc , &gDate , RTC_FORMAT_BIN ) ;
	sprintf ( ctime ,"%02d:%02d:%02d" , gTime.Hours , gTime.Minutes , gTime.Seconds ) ;
	sprintf ( cdate ,"%02d-%02d-%4d" , gDate.Date , gDate.Month , 2000 + gDate.Year ) ;
	send_debug_logs ( ctime ) ;
	send_debug_logs ( cdate ) ;
}

void set_rtc_alarm ( uint8_t Hours , uint8_t Minutes , uint32_t Seconds , uint8_t Date )
{
	RTC_AlarmTypeDef sAlarm ;

	sAlarm.AlarmTime.Hours = Hours ;
	sAlarm.AlarmTime.Minutes = Minutes ;
	sAlarm.AlarmTime.Seconds = Seconds ;
	sAlarm.AlarmTime.SubSeconds = 0x00 ;
	sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
	sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET ;
	sAlarm.AlarmMask = RTC_ALARMMASK_NONE ;
	sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL ;
	sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE ;
	sAlarm.AlarmDateWeekDay = Date ;
	sAlarm.Alarm = RTC_ALARM_A ;

	if ( HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK )
	{
	}
}

void HAL_RTC_AlarmAEventCallback ( RTC_HandleTypeDef *hrtc )
{
	send_debug_logs ( "Alarm Interrupt Occurred\r\n" ) ;
}
