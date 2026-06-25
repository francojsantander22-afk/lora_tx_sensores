#include "cmps2.h"
#include <stdio.h> // Para printf
#include <stdbool.h>

// Variables globales estáticas para almacenar calibración y datos
static I2C_HandleTypeDef *cmps2_i2c;
static float Max[2], Mid[2], Min[2], X, Y;
extern void UART_Print(const char *msg);

void CMPS2_Init(I2C_HandleTypeDef *hi2c) {
    cmps2_i2c = hi2c;
    uint8_t data;

    // Comando registro de control interno 0 para operación SET [cite: 105, 153]
    data = 0x20;
    HAL_I2C_Mem_Write(cmps2_i2c, CMPS2_ADDRESS, 0x07, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    // Comando registro de control interno 1 a resolución de 16 bit, 8ms de tiempo
    data = 0x00;
    HAL_I2C_Mem_Write(cmps2_i2c, CMPS2_ADDRESS, 0x08, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    // Inicializar valores de calibración
    for (int i = 0; i < 2; i++) {
        Max[i] = -32768.0f;
        Min[i] = 32767.0f;
        Mid[i] = 0.0f;
    }
}

void CMPS2_Set(bool reset) {
    uint8_t data;

    // Recarga del capacitor [cite: 155, 156]
    data = 0x80;
    HAL_I2C_Mem_Write(cmps2_i2c, CMPS2_ADDRESS, 0x07, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(50);

    if (reset) {
        data = 0x40; // Reset [cite: 230, 231]
        HAL_I2C_Mem_Write(cmps2_i2c, CMPS2_ADDRESS, 0x07, 1, &data, 1, HAL_MAX_DELAY);
    } else {
        data = 0x20; // Set [cite: 171]
        HAL_I2C_Mem_Write(cmps2_i2c, CMPS2_ADDRESS, 0x07, 1, &data, 1, HAL_MAX_DELAY);
    }
    HAL_Delay(10);
}

void CMPS2_Read_XYZ(void) {
    uint8_t data = 0x01; // Iniciar medición [cite: 108, 109]
    uint8_t status = 0;
    uint8_t tmp[6] = {0};

    HAL_I2C_Mem_Write(cmps2_i2c, CMPS2_ADDRESS, 0x07, 1, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(8); // Esperar que la medición termine (mínimo 7.92 ms [cite: 110])

    // Esperar a que los datos estén listos
    bool flag = false;
    while (!flag) {
        HAL_I2C_Mem_Read(cmps2_i2c, CMPS2_ADDRESS, 0x06, 1, &status, 1, HAL_MAX_DELAY);
        if ((status & 0x01) != 0) {
            flag = true;
        }
    }

    // Leer 6 bytes de datos (X, Y, Z LSB/MSB) [cite: 61, 62, 131, 139]
    HAL_I2C_Mem_Read(cmps2_i2c, CMPS2_ADDRESS, 0x00, 1, tmp, 6, HAL_MAX_DELAY);

    float measured_data[2];

    // Reconstruir datos crudos (Little Endian -> LSB primero, MSB después) [cite: 62]
    int16_t x_raw = (int16_t)((tmp[1] << 8) | tmp[0]);
    int16_t y_raw = (int16_t)((tmp[3] << 8) | tmp[2]);

    measured_data[0] = (float)x_raw;
    measured_data[1] = (float)y_raw;

    // Convertir a mG (0.48828125 mG por LSB) [cite: 285, 287]
    for (int i = 0; i < 2; i++) {
        measured_data[i] = 0.48828125f * measured_data[i];
    }

    X = measured_data[0];
    Y = measured_data[1];

    // Autocalibración contínua de Máximos y Mínimos
    if (measured_data[0] > Max[0]) Max[0] = measured_data[0];
    if (measured_data[0] < Min[0]) Min[0] = measured_data[0];
    if (measured_data[1] > Max[1]) Max[1] = measured_data[1];
    if (measured_data[1] < Min[1]) Min[1] = measured_data[1];

    for (int i = 0; i < 2; i++) {
        Mid[i] = (Max[i] + Min[i]) / 2.0f;
    }
}

float CMPS2_GetHeading(void) {
    float components[2];

    CMPS2_Set(false);   // Polaridad normal
    CMPS2_Read_XYZ();
    components[0] = X;
    components[1] = Y;

    CMPS2_Set(true);    // Polaridad invertida
    CMPS2_Read_XYZ();

    // Eliminar offset [cite: 278]
    components[0] = (components[0] - X) / 2.0f;
    components[1] = (components[1] - Y) / 2.0f;

    float temp0 = 0.0f, temp1 = 0.0f, deg = 0.0f;
    const float RAD_TO_DEG = (180.0f / 3.14159265f);

    // Calcular "heading" por cuadrantes [cite: 293, 294]
    if (components[0] < Mid[0]) {
        if (components[1] > Mid[1]) {
            temp0 = components[1] - Mid[1];
            temp1 = Mid[0] - components[0];
            deg = 90.0f - atanf(temp0 / temp1) * RAD_TO_DEG;
        } else {
            temp0 = Mid[1] - components[1];
            temp1 = Mid[0] - components[0];
            deg = 90.0f + atanf(temp0 / temp1) * RAD_TO_DEG;
        }
    } else {
        if (components[1] < Mid[1]) {
            temp0 = Mid[1] - components[1];
            temp1 = components[0] - Mid[0];
            deg = 270.0f - atanf(temp0 / temp1) * RAD_TO_DEG;
        } else {
            temp0 = components[1] - Mid[1];
            temp1 = components[0] - Mid[0];
            deg = 270.0f + atanf(temp0 / temp1) * RAD_TO_DEG;
        }
    }

    // Aplicar declinación
    deg += DECLINATION;

    if (DECLINATION > 0.0f) {
        if (deg > 360.0f) deg -= 360.0f;
    } else {
        if (deg < 0.0f) deg += 360.0f;
    }

    return deg;
}

const char* CMPS2_DecodeHeading(float measured_angle) {
    // Clasificar el ángulo según datasheet [cite: 298, 299, 300, 301, 302, 308, 309, 310]
    if (measured_angle > 337.25f || measured_angle < 22.5f) {
    	return "Norte\r\n";
    } else if (measured_angle > 292.5f) {
    	return"Noroeste\r\n";
    } else if (measured_angle > 247.5f) {
    	return"Oeste\r\n";
    } else if (measured_angle > 202.5f) {
    	return"Suroeste\r\n";
    } else if (measured_angle > 157.5f) {
    	return"Sur\r\n";
    } else if (measured_angle > 112.5f) {
    	return"Sureste\r\n";
    } else if (measured_angle > 67.5f) {
    	return"Este\r\n";
    } else {
    	return"Noreste\r\n";
    }
}
