/*!
** @file rtc_lld.c
** @version 01.07
** @brief
**         This component implements a real time clock (RTC). Actual date may also be
**         obtained and an alarm function is included.
*/         
/*!
**  @addtogroup RTC1_module RTC1 module documentation
**  @{
*/         

#include "rtc.h"
#include <stdio.h>

void rtc_lld_init(void)
{
  /* If the RTC oscillator is not enabled, get it started now */
  if (!(RTC->CR & RTC_CR_OSCE))
  {
    // Crystal load = 12.5pF, Set load capacitance = 22pF 
    RTC->CR = RTC_CR_SC16P | RTC_CR_SC4P | RTC_CR_SC2P|
              RTC_CR_OSCE|RTC_CR_UM|RTC_CR_SUP;			
			
    // If RTC has invalid time -> reset RTC
    if(RTC->SR & RTC_SR_TIF)
    {
      RTC->SR  = 0x00U;                     /* Disable counter */
      RTC->TPR = RTC_TPR_TPR(0x00);         /* Set prescaler register */
      RTC->TSR = RTC_TSR_TSR(RTC_InitTime);
      RTC->TAR = RTC_TAR_TAR(0x00);         /* Set alarm register - disable alarm */
      RTC->TCR = RTC_TCR_CIC(0x00) |        /* Set compensation */
		             RTC_TCR_TCV(0x00) |
		             RTC_TCR_CIR(0x00) | 
		             RTC_TCR_TCR(0x00);   				                                	
				
      RTC->SR = RTC_SR_TCE;                 /* Enable counter */		
      RTC->IER = 0;                         // Turn off interrupts
     }
  }	
}

uint32_t rtc_lld_getsec(void)
{
  return(RTC->TSR);
}

void rtc_lld_setsec(uint32_t Seconds)
{
  RTC->SR = 0x00U;                          /* Disable counter */
  RTC->TPR = RTC_TPR_TPR(0x00);             /* Set prescaler register */
  RTC->TSR = Seconds;                       /* Set seconds counter */
  RTC->SR = RTC_SR_TCE;                     /* Enable counter */	
}

/*
** ===================================================================
**     Method      :  RTC1_GetTime (component RTC_LDD)
*/
/*!
**     @brief
**         Returns actual time and date. 
**         Note: Fields not supported by HW are calculated in software.
**     @param
**         DeviceDataPtr   - Pointer to device data
**                           structure pointer returned by [Init] method.
**     @param
**         TimePtr         - Pointer to the time structure to
**                           fill with current time.
*/
/* ===================================================================*/
void rtc_lld_get_time(RTCDriver *rtcp, RTCTime *TimePtr)
{
	(void)rtcp;
	
  to_Calendar(TimePtr,rtc_lld_getsec());
}

/*
** ===================================================================
**     Method      :  RTC_SetTime (component RTC_LDD)
*/
/*!
**     @brief
**         Sets new time and date.
**         Note: All LDD_RTC_TTime structure members must be correctly
**         filled in.
**     @param
**         DeviceDataPtr   - Pointer to device data
**                           structure pointer returned by [Init] method.
**     @param
**         TimePtr         - Pointer to the time structure with
**                           new time to set.
**     @return
*/
/* ===================================================================*/
void rtc_lld_set_time(RTCDriver *rtcp,RTCTime *TimePtr) 
{
  rtc_lld_setsec(to_Seconds(TimePtr));
 }

// From SDCC library: time.c
/*-------------------------------------------------------------------------
   time.c - stdlib time conversion routines

   Copyright (C) 2001, Johan Knol <johan.knol AT iduna.nl>

   This library is free software; you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the
   Free Software Foundation; either version 2, or (at your option) any
   later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License 
   along with this library; see the file COPYING. If not, write to the
   Free Software Foundation, 51 Franklin Street, Fifth Floor, Boston,
   MA 02110-1301, USA.

   As a special exception, if you link this library with other files,
   some of which are compiled with SDCC, to produce an executable,
   this library does not by itself cause the resulting executable to
   be covered by the GNU General Public License. This exception does
   not however invalidate any other reasons why the executable file
   might be covered by the GNU General Public License.
-------------------------------------------------------------------------*/

// please note that the tm structure has the years since 1900,
// but time returns the seconds since 1970

/* You need some kind of real time clock for the time() function.
   Either a rtc-chip or some kind of DCF device will do. For TINI, the 
   HAVE_RTC is defined in tinibios.h
   If not, the conversion routines still work.
*/

static uint8_t monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31};
#define LEAP_YEAR(year) ((((year%4)==0) && ((year%100)!=0)) || ((year%400)==0)) 

// return the calendar time, seconds since the Epoch (Jan 1 1970 00:00:00)
// convert broken time to calendar time (seconds since 1970)
uint32_t to_Seconds(RTCTime *timeptr)
{
    int year=timeptr->Year, month=timeptr->Month, i;
    long seconds;

    seconds= (year-RTC_Year_Offset)*(60*60*24L*365);

    // add extra days for leap years
    for (i=RTC_Year_Offset; i<year; i++)
      if (LEAP_YEAR(i))
	      seconds+= 60*60*24L;

    // add days for this year
    for (i=0; i<month; i++)
      if (i==1 && LEAP_YEAR(year))  
	      seconds+= 60*60*24L*29;
      else
	      seconds+= 60*60*24L*monthDays[i];

    seconds+= (timeptr->Day-1)*60*60*24L;
    seconds+= timeptr->Hour*60*60L;
    seconds+= timeptr->Minute*60;
    seconds+= timeptr->Second;
    return seconds;
}

/* convert calendar time (seconds since 1970) to broken-time
*/

RTCTime *to_Calendar(RTCTime *timep, uint32_t Seconds)
{ uint32_t epoch=Seconds;
  uint16_t year;
  uint8_t month, monthLength;
  uint32_t days;
  
  timep->Second=epoch%60;
  epoch/=60; // now it is minutes
  timep->Minute=Seconds%60;
  epoch/=60; // now it is hours
  timep->Hour=Seconds%24;
  epoch/=24; // now it is days
  timep->DayOfWeek=(epoch+RTC_DayofWeek_Offset)%7;
  
  year=1970;
  days=0;
  while((days += (LEAP_YEAR(year) ? 366 : 365)) <= epoch)
    year++;

  timep->Year=year;
  
  days -= LEAP_YEAR(year) ? 366 : 365;
  epoch -= days; // now it is days in this year, starting at 0
  timep->Day=epoch;
  
  days=0;
  month=0;
  monthLength=0;
  for (month=0; month<12; month++)
	{
    if (month==1)
		{ // februari
      if (LEAP_YEAR(year))
	      monthLength=29;

			else
 	      monthLength=28;
    } 
		else
      monthLength = monthDays[month];
    
    if (epoch>=monthLength)
      epoch-=monthLength;
    else
	    break;
  }
  timep->Month=month;
  timep->Day=epoch+1;
  
  return timep;
}

/* END rtc_lld.c */

