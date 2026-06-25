#ifndef INC_CMPS2_H_
#define INC_CMPS2_H_

#include "stm32wlxx_hal.h" // Ajusta esto si tu familia HAL es distinta
#include <math.h>
#include <stdbool.h>
// Dirección I2C desplazada a 8 bits (0x30 << 1)
#define CMPS2_ADDRESS (0x30 << 1)

// Declinación magnética (ajústala según tu ubicación)
#define DECLINATION 6.11f

// Prototipos de funciones
void CMPS2_Init(I2C_HandleTypeDef *hi2c);
void CMPS2_Set(bool reset);
void CMPS2_Read_XYZ(void);
float CMPS2_GetHeading(void);
const char* CMPS2_DecodeHeading(float measured_angle);

#endif /* INC_CMPS2_H_ */
