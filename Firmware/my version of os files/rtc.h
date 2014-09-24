/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
   Concepts and parts of this file have been contributed by Uladzimir Pylinsky
   aka barthess.
   Modified made by  K. C. Lee for MK22DX platform
 */

/**
 * @file    MK22DX/rtc_lld.h
 * @brief   MK22DX RTC low level driver header.
 *
 * @addtogroup RTC
 * @{
 */

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

#include "stdint.h"
#include "board.h"

#ifndef _RTC_LLD_H_
#define _RTC_LLD_H_

#define RTC_InitTime					0x02 	/* Set second register - 2014-01-01 0:0:1 */
#define RTC_Year_Offset					2014
#define RTC_Year_Max					2099
#define RTC_Day_Offset					1U
#define RTC_DayofWeek_Offset				3U

typedef uint32_t RTCDriver;
/**
 * @brief   Structure representing an RTC time stamp.
 */
typedef struct
{ 
  uint16_t Year;
  uint8_t Month;
  uint8_t Day;
  uint8_t DayOfWeek;
  uint8_t Hour;
  uint8_t Minute;
  uint8_t Second;
} RTCTime;

void rtc_lld_init(void);
uint32_t rtc_lld_getsec(void);
void rtc_lld_setsec(uint32_t Seconds);

void rtc_lld_set_time(RTCDriver *rtcp, RTCTime *timespec);
void rtc_lld_get_time(RTCDriver *rtcp, RTCTime *timespec);
uint32_t to_Seconds(RTCTime *timeptr);
RTCTime *to_Calendar(RTCTime *timep, uint32_t Seconds);
#endif /* _RTC_LLD_H_ */

/** @} */
