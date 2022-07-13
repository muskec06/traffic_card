/*
 * fuel_rtc.h
 *
 *  Created on: 23 Kas 2020
 *      Author: muskec
 */

#ifndef _UTIL_RTC_H_
#define _UTIL_RTC_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <string.h>

extern I2C_HandleTypeDef hi2c1;

/* Private define ------------------------------------------------------------*/
#define RTC_DEV_ADDRESS (0x68<<1)  //(0x68)<<1 => 0xD0  RTC ADDRESS


// Convert normal decimal numbers to binary coded decimal
uint8_t decToBcd(int val)
{
  return (uint8_t)( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
int bcdToDec(uint8_t val)
{
  return (int)( (val/16*10) + (val%16) );
}

// function to set time
void setTimeByVal(uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year)
{
	uint8_t set_time[7];
	set_time[0] = decToBcd(sec);
	set_time[1] = decToBcd(min);
	set_time[2] = decToBcd(hour);
	set_time[3] = decToBcd(dow);
	set_time[4] = decToBcd(dom);
	set_time[5] = decToBcd(month);
	set_time[6] = decToBcd(year);

	HAL_I2C_Mem_Write(&hi2c1, RTC_DEV_ADDRESS, 0x00, 1, set_time, 7, 1000);
}


void setTime(Time_t *time)
{
	uint8_t set_time[7];
	set_time[0] = decToBcd(time->second);  		//sec
	set_time[1] = decToBcd(time->minute);  		//min
	set_time[2] = decToBcd(time->hour); 		//hour
	set_time[3] = decToBcd(time->dayofweek);	//dow
	set_time[4] = decToBcd(time->dayofmonth);	//dom
	set_time[5] = decToBcd(time->month);		//month
	set_time[6] = decToBcd(time->year); 		//year

	HAL_I2C_Mem_Write(&hi2c1, RTC_DEV_ADDRESS, 0x00, 1, set_time, 7, 1000);
}

//Time_t time;
void getTime(Time_t *time)
{
	uint8_t get_time[7];
	HAL_I2C_Mem_Read(&hi2c1, RTC_DEV_ADDRESS, 0x00, 1, get_time, 7, 1000);

	time->second = bcdToDec(get_time[0]);
	time->minute = bcdToDec(get_time[1]);
	time->hour = bcdToDec(get_time[2]);
	time->dayofweek = bcdToDec(get_time[3]);
	time->dayofmonth = bcdToDec(get_time[4]);
	time->month = bcdToDec(get_time[5]);
	time->year =  bcdToDec(get_time[6]);
}

void TestRTC()
{
	Time_t time = {
		.second = 10,
		.minute = 7,
		.hour = 15,
		.dayofweek = 1,
		.dayofmonth = 23,
		.month = 11,
		.year = 20
	};

	//setTime(&time);

	//HAL_Delay(10000);
	getTime(&time);
	//printf("%02d:%02d:%02d ", time.hour, time.minute, time.second);
	//printf("%02d-%02d-20%02d\r\n", time.dayofmonth, time.month, time.year);
}

#ifdef __cplusplus
}
#endif

#endif /* _UTIL_RTC_H_ */
