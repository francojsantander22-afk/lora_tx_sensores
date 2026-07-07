/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "app_subghz_phy.h"
#include "usart.h"
#include "gpio.h"
#include "stm32_seq.h"
#include "utilities_def.h"
#include "adc.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <bmi270_config.h>
#include <stdbool.h>
#include "cmps2.h"
#include "ms5611.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* --- Registros BMI270 --- */
#define BMI270_I2C_ADDR    	(0x68 << 1)
#define REG_CHIP_ID         0x00
#define REG_PWR_CONF        0x7C
#define REG_INIT_CTRL       0x59
#define REG_INIT_DATA       0x5E
#define REG_INTERNAL_STATUS 0x21
#define REG_PWR_CTRL        0x7D
#define REG_ACC_CONF        0x40
#define REG_ACC_RANGE       0x41
#define REG_DATA_8          0x0C
#define REG_TEMP_LSB        0x22
//Direcciones SHT21
#define SHT21_I2C_ADDR   (0x40 << 1) // Dirección I2C desplazada 1 bit
#define CMD_MEASURE_T    0xF3        // Trigger T measurement (no hold master)
#define CMD_MEASURE_RH   0xF5        // Trigger RH measurement (no hold master)
//baterias
#define VREF              3.3f
#define ADC_BITS          4096
#define R1                100000.0f
#define R2                220000.0f
#define NUM_MUESTRAS      10
#define INTERVALO_MS      500
#define TEMP_ON           10.0f
#define TEMP_OFF          20.0f
#define ALPHA  0.2f   // factor del filtro: 0.1 = muy suave, 0.3 = más rápido

#define OW_PORT   GPIOA
#define OW_PIN    GPIO_PIN_0
#define OW_LOW()      (OW_PORT->BRR  = OW_PIN)            // tira la línea a 0
#define OW_RELEASE()  (OW_PORT->BSRR = OW_PIN)           // libera (sube por el pull-up)
#define OW_READ()     ((OW_PORT->IDR & OW_PIN) ? 1U : 0U)
/* --- Estructura de datos GPS parseados --- */
typedef struct {
	uint8_t valid; /* 1 = fix confirmado, 0 = sin fix */
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	int32_t lat; /* Grados decimales × 100000 */
	int32_t lon;
	int16_t altitude; /* Metros sobre el nivel del mar */
} GpsData_t;

/* --- Estructura de datos IMU --- */
typedef struct {
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	int16_t gyr_x;
	int16_t gyr_y;
	int16_t gyr_z;
	float temp_BMI270;
} BMI270_Data_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern ADC_HandleTypeDef hadc;

/* --- Buffer circular UART para GPS --- */
#define RX_BUF_SIZE 512
extern volatile uint8_t rx_buffer[RX_BUF_SIZE];
extern volatile uint16_t rx_head;
extern volatile uint16_t rx_tail;
volatile uint8_t tlv_ready = 0;
extern volatile uint8_t rx_byte; // byte temporal que llena la ISR
volatile uint8_t lora_busy = 0;

/* --- Buffer de línea NMEA --- */
char line_buffer[100];
uint8_t line_index = 0;

/* --- Buffer TLV para transmisión LoRa --- */
uint8_t lora_tx_buffer[128];
uint8_t lora_tx_len = 0;

static float vbatFiltrado = 0.0f;

typedef struct {
	float voltaje; uint8_t porcentaje; }
PuntoSoC;

static const PuntoSoC tablaSoC[] = {
  { 4.20f, 100 }, { 4.10f, 90 }, { 4.00f, 80 },
  { 3.90f,  70 }, { 3.80f, 60 }, { 3.70f, 50 },
  { 3.60f,  40 }, { 3.50f, 30 }, { 3.40f, 20 },
  { 3.30f,   10 }, { 3.00f,  0 },
};
static const uint8_t PUNTOS_SOC = sizeof(tablaSoC) / sizeof(tablaSoC[0]);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* Utilidades UART */
void UART_Print(const char *msg);

/* BMI270 */
void BMI270_WriteReg(uint8_t reg, uint8_t val);
uint8_t BMI270_ReadReg(uint8_t reg);
void BMI270_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len);
HAL_StatusTypeDef BMI270_LoadConfigFile(void);
float BMI270_ReadTemperature(void);
/*ahora 1/7*/
float leerTemperaturaDS18B20(void);
void     controlheat(float temp);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* =========================================================
 *  UART
 * ========================================================= */
void UART_Print(const char *msg) {
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
}

/* =========================================================
 *  BMI270 – acceso I2C
 * ========================================================= */
void BMI270_WriteReg(uint8_t reg, uint8_t val) {
	char dbg[70];
	snprintf(dbg, sizeof(dbg), "   [I2C] Escribiendo registro 0x%02X... ", reg);
	UART_Print(dbg);

	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c3, BMI270_I2C_ADDR, reg,
	I2C_MEMADD_SIZE_8BIT, &val, 1, 100);

	snprintf(dbg, sizeof(dbg), "Status HAL: %d\r\n", status);
	UART_Print(dbg);

	for (volatile int k = 0; k < 50000; k++)
		;
}

uint8_t BMI270_ReadReg(uint8_t reg) {
	uint8_t val = 0;
	HAL_I2C_Mem_Read(&hi2c3, BMI270_I2C_ADDR, reg,
	I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
	return val;
}

void BMI270_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len) {
	HAL_I2C_Mem_Read(&hi2c3, BMI270_I2C_ADDR, reg,
	I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

HAL_StatusTypeDef BMI270_LoadConfigFile(void) {
	UART_Print("   -> Configurando punteros 0x5B y 0x5C...\r\n");
	BMI270_WriteReg(0x5B, 0x00);
	BMI270_WriteReg(0x5C, 0x00);

	UART_Print("   -> Enviando 8KB de un solo golpe (Burst Write)...\r\n");

	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c3, BMI270_I2C_ADDR,
	REG_INIT_DATA, I2C_MEMADD_SIZE_8BIT, (uint8_t*) bmi270_config_file,
			bmi270_config_file_size, 1000);

	if (status != HAL_OK) {
		char err[60];
		snprintf(err, sizeof(err),	"   [!] Error I2C en burst write. Codigo HAL: %d\r\n", status);
		UART_Print(err);
		return status;
	}

	UART_Print("   -> Transmision completada.\r\n");
	return HAL_OK;
}

float BMI270_ReadTemperature(void) {
	uint8_t raw[2];
	BMI270_ReadRegs(REG_TEMP_LSB, raw, 2);
	int16_t raw_temp = (int16_t) ((raw[1] << 8) | raw[0]);
	return (raw_temp / 512.0f) + 23.0f;
}
/* =========================================================
 *  STH21 – lectura temperatura + humedad
 * ========================================================= */
void SHT21_Read(float *temperature, float *humidity) {
	uint8_t cmd;
	uint8_t data[3];
	uint16_t raw_val;
	// 1. Leer Temperatura
	cmd = CMD_MEASURE_T;
	HAL_I2C_Master_Transmit(&hi2c3, SHT21_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY); // Enviamos el comando de medición de temperatura
	HAL_Delay(85); // Esperamos el tiempo máximo de conversión para 14 bits (85 ms)
	HAL_I2C_Master_Receive(&hi2c3, SHT21_I2C_ADDR, data, 3, HAL_MAX_DELAY); // Leemos 3 bytes: MSB, LSB y CRC
	// Unimos los bytes y ponemos a '0' los últimos 2 bits de estado (0xFC = 11111100 en binario)
	raw_val = (data[0] << 8) | (data[1] & 0xFC);
	*temperature = -46.85 + 175.72 * ((float) raw_val / 65536.0); // Aplicamos la fórmula del datasheet
	// 2. Leer Humedad Relativa
	cmd = CMD_MEASURE_RH;
	HAL_I2C_Master_Transmit(&hi2c3, SHT21_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY); // Enviamos el comando de medición de humedad
	HAL_Delay(29); // Esperamos el tiempo máximo de conversión para 12 bits (29 ms)
	HAL_I2C_Master_Receive(&hi2c3, SHT21_I2C_ADDR, data, 3, HAL_MAX_DELAY); // Leemos 3 bytes: MSB, LSB y CRC
	raw_val = (data[0] << 8) | (data[1] & 0xFC); // Unimos los bytes y ponemos a '0' los últimos 2 bits de estado
	*humidity = -6.0 + 125.0 * ((float) raw_val / 65536.0);	// Aplicamos la fórmula del datasheet
}
uint32_t leerCanalADC(uint32_t canal) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = canal;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	HAL_ADC_ConfigChannel(&hadc, &sConfig);

	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 100);
	uint32_t valor = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);

	return valor;
}
float leerVoltajeBateria(void) {
	uint32_t suma = 0;
	for (uint8_t i = 0; i < NUM_MUESTRAS; i++) {
		suma += leerCanalADC(ADC_CHANNEL_4); /* PB2 = ADC_IN4 */
		HAL_Delay(5);
	}
	float rawProm = (float) suma / NUM_MUESTRAS;
	float vPin = (rawProm / ADC_BITS) * VREF;
	return vPin * (R1 + R2) / R2;
}
uint8_t voltajeASoC(float voltaje) {
	if (voltaje >= tablaSoC[0].voltaje)
		return 100;
	if (voltaje <= tablaSoC[PUNTOS_SOC - 1].voltaje)
		return 0;

	for (uint8_t i = 0; i < PUNTOS_SOC - 1; i++) {
		if (voltaje <= tablaSoC[i].voltaje
				&& voltaje >= tablaSoC[i + 1].voltaje) {
			float rango_v = tablaSoC[i].voltaje - tablaSoC[i + 1].voltaje;
			float rango_p = tablaSoC[i].porcentaje - tablaSoC[i + 1].porcentaje;
			float fraccion = (voltaje - tablaSoC[i + 1].voltaje) / rango_v;
			return (uint8_t) (tablaSoC[i + 1].porcentaje + fraccion * rango_p);
		}
	}
	return 0;
}
/* =========================================================
 *  GPS – parseo NMEA robusto
 * ========================================================= */
void get_nmea_field(const char *nmea, uint8_t field_num, char *result) {
	uint8_t comma_count = 0;
	uint8_t i = 0, j = 0;
	while (nmea[i] != '\0' && nmea[i] != '*' && comma_count <= field_num) {
		if (nmea[i] == ',') {
			comma_count++;
		} else if (comma_count == field_num) {
			result[j++] = nmea[i];
		}
		i++;
	}
	result[j] = '\0';
}
//empaquetar 1 byte
void tlv_pack_8(uint8_t type, uint8_t val) {
	lora_tx_buffer[lora_tx_len++] = type;
	lora_tx_buffer[lora_tx_len++] = 1; // Longitud: 1 byte
	lora_tx_buffer[lora_tx_len++] = val;
}
// Empaquetar 2 Bytes
void tlv_pack_16(uint8_t type, uint16_t val) {
	lora_tx_buffer[lora_tx_len++] = type;
	lora_tx_buffer[lora_tx_len++] = 2; // Longitud: 2 bytes
	lora_tx_buffer[lora_tx_len++] = (val >> 8) & 0xFF; // MSB
	lora_tx_buffer[lora_tx_len++] = val & 0xFF;        // LSB
}

// Empaquetar 4 Bytes
void tlv_pack_32(uint8_t type, uint32_t val) {
	lora_tx_buffer[lora_tx_len++] = type;
	lora_tx_buffer[lora_tx_len++] = 4; // Longitud: 4 bytes
	lora_tx_buffer[lora_tx_len++] = (val >> 24) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = (val >> 16) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = (val >> 8) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = val & 0xFF;
}

void tlv_pack_time(uint8_t type, uint8_t h, uint8_t m, uint8_t s) {
	lora_tx_buffer[lora_tx_len++] = type;
	lora_tx_buffer[lora_tx_len++] = 3; // Longitud: 3 bytes
	lora_tx_buffer[lora_tx_len++] = h;
	lora_tx_buffer[lora_tx_len++] = m;
	lora_tx_buffer[lora_tx_len++] = s;
}

void tlv_pack_xyz(uint8_t type, int16_t x, int16_t y, int16_t z) {
	lora_tx_buffer[lora_tx_len++] = type;
	lora_tx_buffer[lora_tx_len++] = 6;
	lora_tx_buffer[lora_tx_len++] = (x >> 8) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = x & 0xFF;
	lora_tx_buffer[lora_tx_len++] = (y >> 8) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = y & 0xFF;
	lora_tx_buffer[lora_tx_len++] = (z >> 8) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = z & 0xFF;
}

void tlv_pack_gps_coords(uint8_t type, int32_t lat, int32_t lon) {
	lora_tx_buffer[lora_tx_len++] = type;
	lora_tx_buffer[lora_tx_len++] = 8;
	lora_tx_buffer[lora_tx_len++] = (lat >> 24) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = (lat >> 16) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = (lat >> 8) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = lat & 0xFF;
	lora_tx_buffer[lora_tx_len++] = (lon >> 24) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = (lon >> 16) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = (lon >> 8) & 0xFF;
	lora_tx_buffer[lora_tx_len++] = lon & 0xFF;
}

int32_t nmea_to_int32(char *coord_str, char dir) {
	if (strlen(coord_str) == 0)
		return 0;
	float raw = atof(coord_str);
	int degrees = (int) (raw / 100.0f);
	float minutes = raw - (degrees * 100.0f);
	float decimal_deg = degrees + (minutes / 60.0f);
	if (dir == 'S' || dir == 'W')
		decimal_deg *= -1.0f;
	return (int32_t) (decimal_deg * 100000.0f);
}

/* =========================================================
 *  IMU – lectura BMI270 (accel + gyro + temp)
 * ========================================================= */
void BMI270_Get_Data(BMI270_Data_t *imu_data) {
	uint8_t raw[14];
	BMI270_ReadRegs(REG_DATA_8, raw, 14);
	// Guardamos los datos dentro de la estructura usando la flecha (->)
	imu_data->acc_x = (int16_t) ((raw[1] << 8) | raw[0]);
	imu_data->acc_y = (int16_t) ((raw[3] << 8) | raw[2]);
	imu_data->acc_z = (int16_t) ((raw[5] << 8) | raw[4]);

	imu_data->gyr_x = (int16_t) ((raw[7] << 8) | raw[6]);
	imu_data->gyr_y = (int16_t) ((raw[9] << 8) | raw[8]);
	imu_data->gyr_z = (int16_t) ((raw[11] << 8) | raw[10]);

	imu_data->temp_BMI270 = BMI270_ReadTemperature();
}
// Función maestra para armar el paquete LoRa
uint8_t build_telemetry_payload(uint8_t gps_has_fix, uint8_t h, uint8_t m,
		uint8_t s, int32_t lat, int32_t lon, int16_t alt, uint16_t speed,
		int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyr_x,
		int16_t gyr_y, int16_t gyr_z, int16_t temp_sht, uint16_t hum_sht,
		uint32_t pressure, uint16_t vbat_mv, uint8_t soc, uint16_t mag_angle,
				int16_t temp_bat, uint8_t heat_on)  {
	lora_tx_len = 0; // Reiniciar el puntero del búfer global

	// 1. Datos GPS (Solo se agregan si hay satélites válidos)
	if (gps_has_fix) {
		tlv_pack_time(0x01, h, m, s);
		tlv_pack_gps_coords(0x02, lat, lon);
		tlv_pack_16(0x03, alt);
		tlv_pack_16(0x04, speed);
	}

	// 2. Datos IMU (Siempre se empaquetan)
	tlv_pack_xyz(0x05, acc_x, acc_y, acc_z);
	tlv_pack_xyz(0x06, gyr_x, gyr_y, gyr_z);
	//tlv_pack_16(0x0A, temp_imu); // Type 0x0A para Temperatura IMU

	// 3. Datos Atmosféricos (Siempre se empaquetan)
	tlv_pack_16(0x07, temp_sht);
	tlv_pack_16(0x08, hum_sht);
	tlv_pack_32(0x09, pressure);
	tlv_pack_16(0x0B, vbat_mv);
	tlv_pack_8(0x0C, soc);
	tlv_pack_16(0x0D, mag_angle);
	tlv_pack_16(0x0E, temp_bat);   // 0x0E = temp batería DS18B20
	tlv_pack_8(0x0F, heat_on);     // 0x0F = estado heater (1=ON, 0=OFF)

	return lora_tx_len; // Devuelve el tamaño final del paquete
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MCU Configuration -------------------------------------------------------*/
	HAL_Init();

	/* USER CODE BEGIN Init */
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SubGHz_Phy_Init();
	MX_I2C3_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_ADC_Init();
	/* USER CODE BEGIN 2 */
	/* Forzar PB13 (heater) como push-pull, por si el .ioc lo dejó en open-drain */
	GPIO_InitTypeDef heat_gpio = {0};
	heat_gpio.Pin   = GPIO_PIN_13;
	heat_gpio.Mode  = GPIO_MODE_OUTPUT_PP;      // Push-Pull
	heat_gpio.Pull  = GPIO_NOPULL;
	heat_gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &heat_gpio);
	HAL_Delay(3000);
	UART_Print("Inicializando sistema...\r\n");

	UART_Print("Escaneando I2C3...\r\n");
	for (uint8_t addr = 1; addr < 128; addr++) {
		if (HAL_I2C_IsDeviceReady(&hi2c3, addr << 1, 1, 10) == HAL_OK) {
			char found[30];
			snprintf(found, sizeof(found), "  Dispositivo en 0x%02X\r\n", addr);
			UART_Print(found);
		}
	}

	CMPS2_Init(&hi2c3);
	MS5611_Init(&hi2c3, 0);

	float temperature_sht21, hum;
	uint8_t heat_on = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);   // PB13 = heater
	float gps_speed_kmh = 0.0f;
	/* --- Inicialización BMI270 --- */
	uint8_t chip_id = BMI270_ReadReg(REG_CHIP_ID);
	;
	/* WORKAROUND: En STM32WL la USART1 requiere doble llamada a Receive_IT
	 * para comenzar a generar interrupciones correctamente.
	 * La segunda llamada retorna HAL_BUSY, lo cual es esperado e inofensivo. Si se hace una sola llamada, el sistema nunca habilita las interrupciones de la UART1*/
	HAL_UART_Receive_IT(&huart1, (uint8_t*) &rx_byte, 1);
	HAL_UART_Receive_IT(&huart1, (uint8_t*) &rx_byte, 1);
	if (chip_id != 0x24) {
		char err[60];
		snprintf(err, sizeof(err),
				"ERROR: BMI270 CHIP_ID=0x%02X (esperado 0x24)\r\n", chip_id);
		UART_Print(err);
	} else {
		BMI270_WriteReg(REG_PWR_CONF, 0x00);
		HAL_Delay(1);
		BMI270_WriteReg(REG_INIT_CTRL, 0x00);
		HAL_Delay(1);
		BMI270_LoadConfigFile();
		HAL_Delay(20);
		BMI270_WriteReg(REG_INIT_CTRL, 0x01);
		HAL_Delay(20);
		BMI270_WriteReg(REG_ACC_CONF, 0xA8);
		HAL_Delay(1);
		BMI270_WriteReg(REG_ACC_RANGE, 0x02);
		HAL_Delay(1);
		BMI270_WriteReg(REG_PWR_CTRL, 0x0E);
		HAL_Delay(1);
		BMI270_WriteReg(REG_PWR_CONF, 0x02);
		HAL_Delay(2);
		BMI270_WriteReg(0x42, 0xA9);
		HAL_Delay(1);
		BMI270_WriteReg(0x43, 0x00);
		HAL_Delay(1);
		UART_Print("BMI270 OK.\r\n");
	}

	uint32_t last_gps_time = HAL_GetTick();
	uint32_t last_imu_time = HAL_GetTick();
	BMI270_Data_t imu;

	  /* Habilitar el contador de ciclos DWT para delay_us */
	  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	  DWT->CYCCNT = 0;
	  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

	  /* Pin del DS18B20 como open-drain (pull-up externo) */
	  GPIO_InitTypeDef ow_gpio = {0};
	  ow_gpio.Pin   = OW_PIN;
	  ow_gpio.Mode  = GPIO_MODE_OUTPUT_OD;
	  ow_gpio.Pull  = GPIO_NOPULL;
	  ow_gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(OW_PORT, &ow_gpio);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		   // UART_Print("loop\r\n");     // <-- TEMPORAL, para diagnóstico
		  //  HAL_Delay(200);              // <-- TEMPORAL, para no inundar

		uint8_t trigger_telemetry = 0;
		uint8_t gps_valid = 0;
		/* Watchdog ISR */
		if (huart1.RxState == HAL_UART_STATE_READY) {
			HAL_UART_Receive_IT(&huart1, (uint8_t*) &rx_byte, 1);
			HAL_UART_Receive_IT(&huart1, (uint8_t*) &rx_byte, 1);
		}
		/* TAREA 1: GPS — solo si no hay paquete pendiente */
		if (rx_head != rx_tail) {
			char c = (char) rx_buffer[rx_tail];
			rx_tail = (rx_tail + 1) % RX_BUF_SIZE;

			if (c == '\n' || line_index >= sizeof(line_buffer) - 1) {
				line_buffer[line_index] = '\0';

				if (strstr(line_buffer, "VTG") != NULL) {
					char speed_str[10] = { 0 }; //campo 7 contiene velocidad en km
					get_nmea_field(line_buffer, 7, speed_str);
					if (strlen(speed_str) > 0) {
						gps_speed_kmh = atof(speed_str);
					}
				} else if (strstr(line_buffer, "GGA") != NULL) {
					last_gps_time = HAL_GetTick();
					gps_valid = 1;
					if (lora_busy == 0) {
						trigger_telemetry = 1;
					}
				}
				line_index = 0;
			} else if (c != '\r') {
				line_buffer[line_index++] = c;
			}
		}

		/* TAREA 2: Watchdog */
		if (lora_busy == 0) {
			if (HAL_GetTick() - last_gps_time > 2000) {
				if (HAL_GetTick() - last_imu_time > 1000) {
					last_imu_time = HAL_GetTick();
					trigger_telemetry = 1;
					gps_valid = 0;
				}
			} else {
				last_imu_time = HAL_GetTick();
			}
		}
		//GPS//
		if (trigger_telemetry == 1 && lora_busy == 0) {
			lora_tx_len = 0;
			lora_busy = 1;

			// Variables NMEA subidas de scope para usarlas en el ASCII print
			char time_str[15] = { 0 }, lat_str[15] = { 0 }, ns[2] = { 0 };
			char lon_str[15] = { 0 }, ew[2] = { 0 }, alt_str[10] = { 0 },
					fix_str[2] = { 0 };
			uint8_t gps_has_fix = 0; // Nuestra bandera lógica
			uint8_t h = 0, m = 0, s = 0;
			uint16_t speed_scaled = 0;
			int32_t lat_int = 0, lon_int = 0;
			int16_t alt_int = 0;

			// 1. GPS (si tiene fix)
			if (gps_valid) {
				get_nmea_field(line_buffer, 1, time_str);
				get_nmea_field(line_buffer, 2, lat_str);
				get_nmea_field(line_buffer, 3, ns);
				get_nmea_field(line_buffer, 4, lon_str);
				get_nmea_field(line_buffer, 5, ew);
				get_nmea_field(line_buffer, 6, fix_str);
				get_nmea_field(line_buffer, 9, alt_str);

				if (fix_str[0] >= '1') {
					gps_has_fix = 1;

					int utc_h = (time_str[0] - '0') * 10 + (time_str[1] - '0');
					h = (utc_h - 3 < 0) ? utc_h - 3 + 24 : utc_h - 3;
					m = (time_str[2] - '0') * 10 + (time_str[3] - '0');
					s = (time_str[4] - '0') * 10 + (time_str[5] - '0');

					lat_int = nmea_to_int32(lat_str, ns[0]);
					lon_int = nmea_to_int32(lon_str, ew[0]);

					alt_int = (int16_t) atof(alt_str);
				}
			}

			//BMI270//
			BMI270_Get_Data(&imu);

			float ax_g = imu.acc_x / 4096.0f;
			float ay_g = imu.acc_y / 4096.0f;
			float az_g = imu.acc_z / 4096.0f;

			float gx_dps = imu.gyr_x / 16.4f;
			float gy_dps = imu.gyr_y / 16.4f;
			float gz_dps = imu.gyr_z / 16.4f;
			//SHT21//
			SHT21_Read(&temperature_sht21, &hum);
			int16_t temp_sht_bits = (int16_t) (temperature_sht21 * 100.0f);
			uint16_t hum_bits = (uint16_t) (hum * 100.0f);

			//MS5611
			float temperature_ms5611 = 0.0f;
			float pressure_pa = 0.0f;
			float pressure_ms5611;
			uint32_t press_bits;

			MS5611_Measure(&hi2c3, &temperature_ms5611, &pressure_pa);
			pressure_ms5611 = pressure_pa / 100.0f;  // convertir a mbar
			press_bits = (uint32_t) (pressure_ms5611 * 100.0f); // si tu TLV espera centÃ©simas de mbar

			float measured_angle = CMPS2_GetHeading();
			const char *direccion_viento = CMPS2_DecodeHeading(measured_angle);
			uint16_t mag_angle_scaled = (uint16_t) (measured_angle * 100.0f);
			speed_scaled = (uint16_t) (gps_speed_kmh * 100.0f);

			float voltajeCrudo = leerVoltajeBateria();

			// Filtro exponencial: suaviza la lectura entre ciclos
			if (vbatFiltrado == 0.0f)
				vbatFiltrado = voltajeCrudo; // inicialización en el primer ciclo
			else
				vbatFiltrado = ALPHA * voltajeCrudo
						+ (1.0f - ALPHA) * vbatFiltrado;

			float voltaje = vbatFiltrado;
			uint8_t soc = voltajeASoC(voltaje);
		      float temp    = leerTemperaturaDS18B20();
		      controlheat(temp);
		      uint8_t heat_on = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);

			uint16_t vbat_mv = (uint16_t) (voltaje * 1000.0f);

			build_telemetry_payload(gps_has_fix, h, m, s, lat_int, lon_int,
					alt_int, speed_scaled, imu.acc_x, imu.acc_y, imu.acc_z,
					imu.gyr_x, imu.gyr_y, imu.gyr_z, temp_sht_bits, hum_bits,
					press_bits, vbat_mv, soc, mag_angle_scaled,
					(int16_t)(temp * 100.0f), heat_on);

			char separador[100] =
					"------------------------------------------------------------------------\r\n";
			UART_Print(separador);
			char debug_msg[200] = "TLV [";
			char hex_byte[5];
			for (int i = 0; i < lora_tx_len; i++) {
				snprintf(hex_byte, sizeof(hex_byte), "%02X ",
						lora_tx_buffer[i]);
				strcat(debug_msg, hex_byte);
			}
			char tail_buf[20];
			snprintf(tail_buf, sizeof(tail_buf), "] %d bytes\r\n", lora_tx_len);
			strcat(debug_msg, tail_buf);
			UART_Print(debug_msg);
			UART_Print(separador);

			char ascii_msg[512];
			// 1. Verificamos si pasaron más de 2 segundos sin datos (Desconectado)
			if (HAL_GetTick() - last_gps_time > 2000) {
				snprintf(ascii_msg, sizeof(ascii_msg),
				        "[GPS] Lat: %s %s, Lon: %s %s, Alt: %s, Vel: %.2f km/h, Hora: %.2d:%.2d:%.2d\r\n"
				        "[IMU] A: %+.2fg %+.2fg %+.2fg | G: %+.1fdps %+.1fdps %+.1fdps | T: %.1fC\r\n"
				        "[SHT21] Temperatura: %.2f C | Humedad: %.2f %%\r\n"
				        "[MS5611] Temperatura: %.2f C | Presión: %.2f\r\n"
				        "[CMPS2]  Ángulo: %.2f ° Dirreción: %s"
				        "[DS18B20] Bateria: %.2f C | Heater: %s\r\n"
				        "VBAT: %.3f V | SOC: %u%%\r\n",
				        lat_str, ns, lon_str, ew, alt_str, gps_speed_kmh, h, m, s,
				        ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, imu.temp_BMI270,
				        temperature_sht21, hum, temperature_ms5611,
				        pressure_ms5611, measured_angle, direccion_viento,
				        temp, heat_on ? "ON" : "OFF", voltaje, soc);
			}
			// 2. Verificamos si hay conexión y tenemos fix satelital
			else if (gps_valid && fix_str[0] >= '1') {
				snprintf(ascii_msg, sizeof(ascii_msg),
				        "[GPS] Lat: %s %s, Lon: %s %s, Alt: %s, Vel: %.2f km/h, Hora: %.2d:%.2d:%.2d\r\n"
				        "[IMU] A: %+.2fg %+.2fg %+.2fg | G: %+.1fdps %+.1fdps %+.1fdps | T: %.1fC\r\n"
				        "[SHT21] Temperatura: %.2f C | Humedad: %.2f %%\r\n"
				        "[MS5611] Temperatura: %.2f C | Presión: %.2f\r\n"
				        "[CMPS2]  Ángulo: %.2f ° Dirreción: %s"
				        "[DS18B20] Bateria: %.2f C | Heater: %s\r\n"
				        "VBAT: %.3f V | SOC: %u%%\r\n",
				        lat_str, ns, lon_str, ew, alt_str, gps_speed_kmh, h, m, s,
				        ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, imu.temp_BMI270,
				        temperature_sht21, hum, temperature_ms5611,
				        pressure_ms5611, measured_angle, direccion_viento,
				        temp, heat_on ? "ON" : "OFF", voltaje, soc);
			}
			// 3. Hay conexión pero aún no hay fix
			else {
				snprintf(ascii_msg, sizeof(ascii_msg),
				        "[GPS] Lat: %s %s, Lon: %s %s, Alt: %s, Vel: %.2f km/h, Hora: %.2d:%.2d:%.2d\r\n"
				        "[IMU] A: %+.2fg %+.2fg %+.2fg | G: %+.1fdps %+.1fdps %+.1fdps | T: %.1fC\r\n"
				        "[SHT21] Temperatura: %.2f C | Humedad: %.2f %%\r\n"
				        "[MS5611] Temperatura: %.2f C | Presión: %.2f\r\n"
				        "[CMPS2]  Ángulo: %.2f ° Dirreción: %s"
				        "[DS18B20] Bateria: %.2f C | Heater: %s\r\n"
				        "VBAT: %.3f V | SOC: %u%%\r\n",
				        lat_str, ns, lon_str, ew, alt_str, gps_speed_kmh, h, m, s,
				        ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, imu.temp_BMI270,
				        temperature_sht21, hum, temperature_ms5611,
				        pressure_ms5611, measured_angle, direccion_viento,
				        temp, heat_on ? "ON" : "OFF", voltaje, soc);
			}
			char separador2[100] =
					"------------------------------------------------------------------------\r\n\r\n";
			UART_Print(ascii_msg);
			UART_Print(separador2);
			tlv_ready = 1;
			UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
					CFG_SEQ_Prio_0);
		}
		MX_SubGHz_Phy_Process();
	}

	/* USER CODE END WHILE */
	/* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE
			| RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3 | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
// Delay de microsegundos con el contador de ciclos DWT (Cortex-M4)
static inline void delay_us(uint32_t us)
{
    uint32_t inicio = DWT->CYCCNT;
    uint32_t ticks  = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - inicio) < ticks) { }
}

// Reset + presencia. Devuelve 1 si hay sensor, 0 si no contesta.
static uint8_t ow_reset(void)
{
    uint8_t presencia;
    OW_LOW();      delay_us(480);
    OW_RELEASE();  delay_us(70);
    presencia = !OW_READ();          // el sensor tira la línea = presencia
    delay_us(410);
    return presencia;
}

static void ow_write_bit(uint8_t bit)
{
    if (bit) { OW_LOW(); delay_us(6);  OW_RELEASE(); delay_us(64); }
    else     { OW_LOW(); delay_us(60); OW_RELEASE(); delay_us(10); }
}

static uint8_t ow_read_bit(void)
{
    uint8_t bit;
    OW_LOW();      delay_us(6);
    OW_RELEASE();  delay_us(9);
    bit = OW_READ();
    delay_us(55);
    return bit;
}

static void ow_write_byte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++) { ow_write_bit(byte & 0x01); byte >>= 1; }
}

static uint8_t ow_read_byte(void)
{
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++) {
        byte >>= 1;
        if (ow_read_bit()) byte |= 0x80;
    }
    return byte;
}

// Lee temperatura en °C. Devuelve -127.0 si no hay sensor.
float leerTemperaturaDS18B20(void)
{
    if (!ow_reset()) return -127.0f;

    __disable_irq();                 // timing estable durante los comandos
    ow_reset();
    ow_write_byte(0xCC);             // Skip ROM (un solo sensor en el bus)
    ow_write_byte(0x44);             // Convert T
    __enable_irq();

    HAL_Delay(750);                  // conversión 12 bits = 750 ms máx

    __disable_irq();
    ow_reset();
    ow_write_byte(0xCC);             // Skip ROM
    ow_write_byte(0xBE);             // Read Scratchpad
    uint8_t lsb = ow_read_byte();
    uint8_t msb = ow_read_byte();
    __enable_irq();

    int16_t raw = (int16_t)((msb << 8) | lsb);
    return (float)raw / 16.0f;       // cada LSB = 0.0625 °C = 1/16
}

void controlheat(float temp)
{
    static uint8_t heaterEncendido = 0;

    if (!heaterEncendido && temp < TEMP_ON) {
        heaterEncendido = 1;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   // PB13
    } else if (heaterEncendido && temp > TEMP_OFF) {
        heaterEncendido = 0;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // PB13
    }
}


/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
