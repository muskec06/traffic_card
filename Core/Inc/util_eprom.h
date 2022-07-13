/*
 * fuel_io.h
 *
 *  Created on: 12 Ekim 2021
 *      Author: muskec
 */

#ifndef _UTIL_EPROM_H_
#define _UTIL_EPROM_H_

#ifdef __cplusplus
 extern "C" {
#endif

#define LAMP_SIZE 40
#define EPROM_DEV_ADDRESS  		(0x54)<<1  //(0x54)<<1 => 0xA8
#define MAGIC_ADDRESS		  	0x0000
#define LAMP_DEFAULT_ADDRESS  	0x0004


typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
} Lamp_t;

Lamp_t flash[LAMP_SIZE];
uint8_t lampDefaultValue[LAMP_SIZE] = {0};
extern I2C_HandleTypeDef hi2c1;

const Lamp_t lamp[LAMP_SIZE] = {
    {.port=O1_1_GPIO_Port, .pin=O1_1_Pin},
    {.port=O2_1_GPIO_Port, .pin=O2_1_Pin},
    {.port=O3_1_GPIO_Port, .pin=O3_1_Pin},
    {.port=O4_1_GPIO_Port, .pin=O4_1_Pin},
    {.port=O5_1_GPIO_Port, .pin=O5_1_Pin},
    {.port=O6_1_GPIO_Port, .pin=O6_1_Pin},
    {.port=O7_1_GPIO_Port, .pin=O7_1_Pin},
    {.port=O8_1_GPIO_Port, .pin=O8_1_Pin},
    {.port=O1_2_GPIO_Port, .pin=O1_2_Pin},
    {.port=O2_2_GPIO_Port, .pin=O2_2_Pin},
    {.port=O3_2_GPIO_Port, .pin=O3_2_Pin},
    {.port=O4_2_GPIO_Port, .pin=O4_2_Pin},
    {.port=O5_2_GPIO_Port, .pin=O5_2_Pin},
    {.port=O6_2_GPIO_Port, .pin=O6_2_Pin},
    {.port=O7_2_GPIO_Port, .pin=O7_2_Pin},
    {.port=O8_2_GPIO_Port, .pin=O8_2_Pin},
    {.port=O1_3_GPIO_Port, .pin=O1_3_Pin},
    {.port=O2_3_GPIO_Port, .pin=O2_3_Pin},
    {.port=O3_3_GPIO_Port, .pin=O3_3_Pin},
    {.port=O4_3_GPIO_Port, .pin=O4_3_Pin},
    {.port=O5_3_GPIO_Port, .pin=O5_3_Pin},
    {.port=O6_3_GPIO_Port, .pin=O6_3_Pin},
    {.port=O7_3_GPIO_Port, .pin=O7_3_Pin},
    {.port=O8_3_GPIO_Port, .pin=O8_3_Pin},
    {.port=O1_4_GPIO_Port, .pin=O1_4_Pin},
    {.port=O2_4_GPIO_Port, .pin=O2_4_Pin},
    {.port=O3_4_GPIO_Port, .pin=O3_4_Pin},
    {.port=O4_4_GPIO_Port, .pin=O4_4_Pin},
    {.port=O5_4_GPIO_Port, .pin=O5_4_Pin},
    {.port=O6_4_GPIO_Port, .pin=O6_4_Pin},
    {.port=O7_4_GPIO_Port, .pin=O7_4_Pin},
    {.port=O8_4_GPIO_Port, .pin=O8_4_Pin},
    {.port=O1_5_GPIO_Port, .pin=O1_5_Pin},
    {.port=O2_5_GPIO_Port, .pin=O2_5_Pin},
    {.port=O3_5_GPIO_Port, .pin=O3_5_Pin},
    {.port=O4_5_GPIO_Port, .pin=O4_5_Pin},
    {.port=O5_5_GPIO_Port, .pin=O5_5_Pin},
    {.port=O6_5_GPIO_Port, .pin=O6_5_Pin},
    {.port=O7_5_GPIO_Port, .pin=O7_5_Pin},
    {.port=O8_5_GPIO_Port, .pin=O8_5_Pin}
};

//Function protypes
int checkMagicData();
void writekMagicData();
void readEpromLampDefault();
uint8_t writeEpromLampDefault(uint8_t *lampMem);

int checkMagicData()
{
	uint32_t magicData;
	uint16_t bufferSize = sizeof(magicData);
	uint8_t *rxBuffer = (uint8_t *) &magicData;  //Convert ADDRESS to (uint8_t *)

	HAL_I2C_Mem_Read(&hi2c1, EPROM_DEV_ADDRESS, MAGIC_ADDRESS, I2C_MEMADD_SIZE_16BIT, rxBuffer, bufferSize, 50);

	if(magicData == 123456789) {
		return 1;
	}
	return 0;
}

void writekMagicData()
{
	uint32_t magicData = 123456789;
	uint16_t bufferSize = sizeof(magicData);
	uint8_t *txBuffer = (uint8_t *) &magicData;  //Convert ADDRESS to (uint8_t *)

	HAL_I2C_Mem_Write(&hi2c1, EPROM_DEV_ADDRESS, MAGIC_ADDRESS, I2C_MEMADD_SIZE_16BIT, txBuffer, bufferSize, 100);
}

void readEpromLampDefault()
{
	uint16_t bufferSize = sizeof(lampDefaultValue);
	uint8_t *rxBuffer = (uint8_t *) lampDefaultValue;  //Convert ADDRESS to (uint8_t *)

	memset(lampDefaultValue,0,sizeof(lampDefaultValue)); //Reset the lamp values to the closed state
	if(checkMagicData() == 1) {
		HAL_I2C_Mem_Read(&hi2c1, EPROM_DEV_ADDRESS, LAMP_DEFAULT_ADDRESS, I2C_MEMADD_SIZE_16BIT, rxBuffer, bufferSize, 100);
	}
}

uint8_t writeEpromLampDefault(uint8_t *lampMem)
{
	HAL_StatusTypeDef status;
	uint8_t *txBuffer = (uint8_t *) lampMem;
	uint16_t bufferSize = sizeof(lampDefaultValue);

	writekMagicData();
	if(checkMagicData() == 1) {
		status = HAL_I2C_Mem_Write(&hi2c1, EPROM_DEV_ADDRESS, LAMP_DEFAULT_ADDRESS, I2C_MEMADD_SIZE_16BIT, txBuffer, bufferSize, 100);
		if(status == HAL_OK) {
			return 1;
		}
	}
	return 0;
}


#ifdef __cplusplus
}
#endif

#endif /* _UTIL_EPROM_H_ */
