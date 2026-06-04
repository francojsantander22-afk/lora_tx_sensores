/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    subghz_phy_app.c
 * @author  MCD Application Team
 * @brief   Application of the SubGHz_Phy Middleware
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
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
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"

/* USER CODE BEGIN Includes */
#include "stm32_timer.h"
#include "stm32_seq.h"
#include "utilities_def.h"
#include "app_version.h"
#include "subghz_phy_version.h"
#include "main.h"
#include <stdio.h>

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
/* Paquete TLV generado en main.c */
extern uint8_t lora_tx_buffer[];
extern uint8_t lora_tx_len;
extern volatile uint8_t tlv_ready;
extern volatile uint8_t rx_byte;
extern volatile uint8_t lora_busy;
/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	RX, RX_TIMEOUT, RX_ERROR, TX, TX_TIMEOUT,
} States_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Configurations */
/*Timeout*/
#define RX_TIMEOUT_VALUE              3000
#define TX_TIMEOUT_VALUE              3000
/* PING string*/
#define PING "PING"
/* PONG string*/
#define PONG "PONG"
/*Size of the payload to be sent*/
/* Size must be greater of equal the PING and PONG*/
#define MAX_APP_BUFFER_SIZE          255
#if (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE)
#error PAYLOAD_LEN must be less or equal than MAX_APP_BUFFER_SIZE
#endif /* (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE) */
/* wait for remote to be in Rx, before sending a Tx frame*/
#define RX_TIME_MARGIN                200
/* Afc bandwidth in Hz */
#define FSK_AFC_BANDWIDTH             83333
/* LED blink Period*/
#define LED_PERIOD_MS                 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Variables de configuración dinámica */
uint32_t Conf_Frequency = 917300000;
uint32_t Conf_Bandwidth = 0;      // 125 kHz
uint32_t Conf_SF = 7;             // Spreading Factor
uint32_t Conf_PayloadLen = 128;     // Por defecto "PING" son 4 bytes
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;

/* USER CODE BEGIN PV */

/*Ping Pong FSM states */
static States_t State = RX;
/* App Rx Buffer*/
static uint8_t BufferRx[MAX_APP_BUFFER_SIZE];
/* App Tx Buffer*/
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];
/* Last  Received Buffer Size*/
uint16_t RxBufferSize = 0;
/* Last  Received packer Rssi*/
int8_t RssiValue = 0;
/* Last  Received packer SNR (in Lora modulation)*/
int8_t SnrValue = 0;
/* Led Timers objects*/
static UTIL_TIMER_Object_t timerLed;
/* device state. Master: true, Slave: false*/
bool isMaster = true;
/* random delay to make sure 2 devices will sync*/
/* the closest the random delays are, the longer it will
 take for the devices to sync when started simultaneously*/
static int32_t random_delay;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(void);

/**
 * @brief Function to be executed on Radio Rx Done event
 * @param  payload ptr of buffer received
 * @param  size buffer size
 * @param  rssi
 * @param  LoraSnr_FskCfo
 */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi,
		int8_t LoraSnr_FskCfo);

/**
 * @brief Function executed on Radio Tx Timeout event
 */
static void OnTxTimeout(void);

/**
 * @brief Function executed on Radio Rx Timeout event
 */
static void OnRxTimeout(void);

/**
 * @brief Function executed on Radio Rx Error event
 */
static void OnRxError(void);

/* USER CODE BEGIN PFP */
/**
 * @brief  Function executed on when led timer elapses
 * @param  context ptr of LED context
 */
static void OnledEvent(void *context);

/**
 * @brief PingPong state machine implementation
 */
static void PingPong_Process(void);

uint8_t Reconfiguracion_Pendiente = 0;

static void APP_Menu(void) {
	uint8_t key = 0;
	APP_LOG(TS_OFF, VLEVEL_M, "\r\n--- CONFIGURACION DE RADIO RAK ---\r\n");

	// 1. Configuración de Frecuencia
	APP_LOG(TS_OFF, VLEVEL_M, "Frecuencia: [1] 915MHz, [2] 917.3MHz\r\n");
	while (1) {
		if (HAL_UART_Receive(&huart2, &key, 1, 1000) == HAL_OK) { // Timeout de 1s
			if (key == '1') {
				Conf_Frequency = 915000000;
				break;
			}
			if (key == '2') {
				Conf_Frequency = 917300000;
				break;
			}
		}
	}

	// 2. Spreading Factor
	APP_LOG(TS_OFF, VLEVEL_M,
			"SF: [7], [8], [9], [A] SF10, [B] SF11, [C] SF12\r\n");
	while (1) {
		if (HAL_UART_Receive(&huart2, &key, 1, 1000) == HAL_OK) {
			if (key >= '7' && key <= '9') {
				Conf_SF = key - '0';
				break;
			}
			if (key == 'a' || key == 'A') {
				Conf_SF = 10;
				break;
			}
			if (key == 'b' || key == 'B') {
				Conf_SF = 11;
				break;
			}
			if (key == 'c' || key == 'C') {
				Conf_SF = 12;
				break;
			}
		}
	}

	// 3. Bandwidth
	APP_LOG(TS_OFF, VLEVEL_M, "BW: [0] 125kHz, [1] 250kHz, [2] 500kHz\r\n");
	while (1) {
		if (HAL_UART_Receive(&huart2, &key, 1, 1000) == HAL_OK) {
			if (key >= '0' && key <= '2') {
				Conf_Bandwidth = key - '0';
				break;
			}
		}
	}

	// 4. Tamaño de Datos
	APP_LOG(TS_OFF, VLEVEL_M, "Payload: [1] 4B, [2] 32B, [3] 64B, [4] 128B\r\n");
	while (1) {
		if (HAL_UART_Receive(&huart2, &key, 1, 1000) == HAL_OK) {
			if (key == '1') {
				Conf_PayloadLen = 4;
				break;
			}
			if (key == '2') {
				Conf_PayloadLen = 32;
				break;
			}
			if (key == '3') {
				Conf_PayloadLen = 64;
				break;
			}
			if (key == '4') {
				Conf_PayloadLen = 128;
				break;
			}
		}
	}
	APP_LOG(TS_OFF, VLEVEL_M, "--- RECONFIGURACION LOCAL LISTA ---\r\n");
	Reconfiguracion_Pendiente = 1; // Activamos la bandera para la FSM
}

void Aplicar_Nuevos_Parametros(void) {
	Radio.Sleep();
	Radio.SetChannel(Conf_Frequency);

	Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, Conf_Bandwidth, Conf_SF,
			LORA_CODINGRATE,
			LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
			true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

	Radio.SetRxConfig(MODEM_LORA, Conf_Bandwidth, Conf_SF,
	LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
	LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0,
			LORA_IQ_INVERSION_ON, true);

	Radio.SetMaxPayloadLength(MODEM_LORA, Conf_PayloadLen);
	APP_LOG(TS_ON, VLEVEL_M, "Radio actualizada: FR=%d | SF=%d | BW=%d\r\n",
			Conf_Frequency, Conf_SF, Conf_Bandwidth);
}

/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void) {
	/* USER CODE BEGIN SubghzApp_Init_1 */

	//UTIL_TIMER_Create(&timerTxCycle, 5000, UTIL_TIMER_ONESHOT, OnTxCycleEvent, NULL);

	APP_LOG(TS_OFF, VLEVEL_M, "\n\rPING PONG\n\r");
	/* Get SubGHY_Phy APP version*/
	APP_LOG(TS_OFF, VLEVEL_M, "APPLICATION_VERSION: V%X.%X.%X\r\n",
			(uint8_t)(APP_VERSION_MAIN), (uint8_t)(APP_VERSION_SUB1),
			(uint8_t)(APP_VERSION_SUB2));

	/* Get MW SubGhz_Phy info */
	APP_LOG(TS_OFF, VLEVEL_M, "MW_RADIO_VERSION:    V%X.%X.%X\r\n",
			(uint8_t)(SUBGHZ_PHY_VERSION_MAIN),
			(uint8_t)(SUBGHZ_PHY_VERSION_SUB1),
			(uint8_t)(SUBGHZ_PHY_VERSION_SUB2));

	//APP_Menu(); Directamente comienza con los valores predefinidos, si se quiere se cambia con "m"

	APP_LOG(TS_OFF, VLEVEL_M, "\n\rSISTEMA CONFIGURADO - INICIANDO MODO TX\n\r");

	/* Led Timers*/
	UTIL_TIMER_Create(&timerLed, LED_PERIOD_MS, UTIL_TIMER_ONESHOT, OnledEvent,
			NULL);
	UTIL_TIMER_Start(&timerLed);
	/* USER CODE END SubghzApp_Init_1 */

	/* Radio initialization */
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.RxDone = OnRxDone;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxTimeout = OnRxTimeout;
	RadioEvents.RxError = OnRxError;

	Radio.Init(&RadioEvents);

	/* USER CODE BEGIN SubghzApp_Init_2 */
	/*calculate random delay for synchronization*/
	random_delay = (Radio.Random()) >> 22; /*10bits random e.g. from 0 to 1023 ms*/

	/* Radio Set frequency */
	/* 1. Aplicar la frecuencia configurada en el menú */
	Radio.SetChannel(Conf_Frequency);

	/* 2. Configuración de Radio LoRa usando variables dinámicas */
#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
	APP_LOG(TS_OFF, VLEVEL_M, "---------------\n\r");
	APP_LOG(TS_OFF, VLEVEL_M, "LORA CONFIGURADA:\n\r");
	APP_LOG(TS_OFF, VLEVEL_M, "FR=%d Hz | SF=%d | BW=%d\n\r", Conf_Frequency,
			Conf_SF, Conf_Bandwidth);

	// Usamos Conf_Bandwidth y Conf_SF en lugar de las macros fijas
	Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, Conf_Bandwidth, Conf_SF,
			LORA_CODINGRATE,
			LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
			true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

	Radio.SetRxConfig(MODEM_LORA, Conf_Bandwidth, Conf_SF,
	LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
	LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0,
			LORA_IQ_INVERSION_ON, true);

	/* 3. Aplicar el tamaño máximo de payload del menú */
	Radio.SetMaxPayloadLength(MODEM_LORA, Conf_PayloadLen);

#endif /* USE_MODEM_LORA */

	/* Limpiar buffers */
	memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);
	memset(BufferRx, 0x0, MAX_APP_BUFFER_SIZE);

	APP_LOG(TS_ON, VLEVEL_L, "Modo TX iniciado...\n\r");

	/* Inicia la escucha */
	State = TX;
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), UTIL_SEQ_RFU,
			PingPong_Process);
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
			CFG_SEQ_Prio_0);
	/* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void) {
	/* USER CODE BEGIN OnTxDone */
	APP_LOG(TS_ON, VLEVEL_L,
			"OnTxDone: Transmision completada. Pasando a escuchar ACK...\n\r");

	/* CAMBIO AQUÍ: Seteamos estado RX y abrimos ventana de escucha en la radio */
	State = RX;
	//Radio.Rx(RX_TIMEOUT_VALUE);

	/* Ejecutamos el proceso en el secuenciador */
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
			CFG_SEQ_Prio_0);

	/* USER CODE END OnTxDone */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi,
		int8_t LoraSnr_FskCfo) {
	/* USER CODE BEGIN OnRxDone */
	APP_LOG(TS_ON, VLEVEL_L, "OnRxDone\n\r");
#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
	APP_LOG(TS_ON, VLEVEL_L, "RssiValue=%d dBm, SnrValue=%ddB\n\r", rssi,
			LoraSnr_FskCfo);
	/* Record payload Signal to noise ratio in Lora*/
	SnrValue = LoraSnr_FskCfo;
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */
#if ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))
  APP_LOG(TS_ON, VLEVEL_L, "RssiValue=%d dBm, Cfo=%dkHz\n\r", rssi, LoraSnr_FskCfo);
  SnrValue = 0; /*not applicable in GFSK*/
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */
	/* Update the State of the FSM*/
	State = RX;
	/* Clear BufferRx*/
	memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
	/* Record payload size*/
	RxBufferSize = size;
	if (RxBufferSize <= MAX_APP_BUFFER_SIZE) {
		memcpy(BufferRx, payload, RxBufferSize);
	}
	/* Record Received Signal Strength*/
	RssiValue = rssi;
	/* Record payload content*/
	APP_LOG(TS_ON, VLEVEL_H, "payload. size=%d \n\r", size);
	for (int i = 0; i < RxBufferSize; i++) {
		APP_LOG(TS_OFF, VLEVEL_H, "%02X ", BufferRx[i]);
		if (i % 16 == 15) {
			APP_LOG(TS_OFF, VLEVEL_H, "\n\r");
		}
	}
	APP_LOG(TS_OFF, VLEVEL_H, "\n\r");
	/* Run PingPong process in background*/
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
			CFG_SEQ_Prio_0);
	/* USER CODE END OnRxDone */
}

static void OnTxTimeout(void) {
	/* USER CODE BEGIN OnTxTimeout */
	APP_LOG(TS_ON, VLEVEL_L, "OnTxTimeout\n\r");
	/* Update the State of the FSM*/
	State = TX_TIMEOUT;
	/* Run PingPong process in background*/
	// UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);

	/* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void) {
	/* USER CODE BEGIN OnRxTimeout */
	APP_LOG(TS_ON, VLEVEL_L, "OnRxTimeout\n\r");
	lora_busy=0;
	/* Update the State of the FSM*/
	State = RX_TIMEOUT;
	/* Run PingPong process in background*/
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
			CFG_SEQ_Prio_0);
	//UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
	/* USER CODE END OnRxTimeout */
}

static void OnRxError(void) {
	/* USER CODE BEGIN OnRxError */
	APP_LOG(TS_ON, VLEVEL_L, "OnRxError\n\r");
	/* Update the State of the FSM*/
	State = RX_ERROR;
	/* Run PingPong process in background*/
	//UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
	/* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */
static void PingPong_Process(void)  //funcion de tx del rak
{
	//Radio.Sleep();
	/* Send the next PING frame */
	/* Add delay between RX and TX*/
	/* add random_delay to force sync between boards after some trials*/

	uint8_t uart_key;
	// Chequeo rápido de UART para el menú
	if (HAL_UART_Receive(&huart2, &uart_key, 1, 10) == HAL_OK) {
		if (uart_key == 'm' || uart_key == 'M') {
			APP_Menu();
			State = TX; // Forzamos a enviar la nueva config [cite: 189]
			UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
					CFG_SEQ_Prio_0);
		}
	}

	switch (State) {
	case RX:
		if (RxBufferSize > 0) {
			if (RxBufferSize < MAX_APP_BUFFER_SIZE) {
				BufferRx[RxBufferSize] = '\0';
			}

			// Si esperábamos el ACK de una reconfiguración
			if (strncmp((const char*) BufferRx, "ACK_CONF", 8) == 0) {
				APP_LOG(TS_ON, VLEVEL_L,
						"--- RX CONFIRMO CAMBIO. APLICANDO PARAMETROS... ---\n\r");
				Aplicar_Nuevos_Parametros();
				Reconfiguracion_Pendiente = 0;
				RxBufferSize = 0;
				HAL_Delay(1000);
				State = TX;
				UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
						CFG_SEQ_Prio_0);
				break;
			}

			// Comportamiento normal: Se recibió el ACK regular del Slave
			APP_LOG(TS_ON, VLEVEL_L, "--- ACK RECIBIDO CON EXITO ---\n\r");
			RxBufferSize = 0;
			//HAL_Delay(3000); // Pausa de 3 segundos antes de mandar el próximo PING regular
			lora_busy=0;
			State = TX;
			UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
					CFG_SEQ_Prio_0);
		} else {
			// Si no hay datos en el buffer, mantenemos la radio escuchando
			Radio.Rx(RX_TIMEOUT_VALUE);
		}
		break;

	case TX:
		memset(BufferTx, 0, MAX_APP_BUFFER_SIZE);

		if (Reconfiguracion_Pendiente) {
			/* Comando de reconfiguración — sin cambios */
			APP_LOG(TS_ON, VLEVEL_L,
					"Enviando comando de reconfiguracion...\n\r");
			sprintf((char*) BufferTx, "CONF:%lu:%lu:%lu:%lu", Conf_Frequency,
					Conf_SF, Conf_Bandwidth, Conf_PayloadLen);
			Radio.Send(BufferTx, strlen((const char*) BufferTx));
		} else if (tlv_ready && lora_tx_len > 0) {
			/* Paquete TLV de telemetría listo */
			APP_LOG(TS_ON, VLEVEL_L, "TX: Enviando TLV (%d bytes)...\n\r",
					lora_tx_len);
			memcpy(BufferTx, lora_tx_buffer, lora_tx_len);
			Radio.Send(BufferTx, lora_tx_len);
			tlv_ready = 0; /* consumido */
		} //else {
			/* Todavía no hay paquete nuevo — esperar y reintentar */
			//APP_LOG(TS_ON, VLEVEL_L, "TX: Esperando paquete TLV...\n\r");
			//HAL_Delay(100);
			//UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
			//UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
		//}
		break;

	case RX_TIMEOUT:
	case TX_TIMEOUT:
	case RX_ERROR:
		/* CAMBIO AQUÍ: Si al Master le da un timeout esperando el ACK, significa que el paquete
		 se perdió o el RX no escuchó. Reintentamos enviando un PING nuevo */
		APP_LOG(TS_ON, VLEVEL_L,
				"Master Alerta: Sin respuesta (Timeout)\n\r\n\r")
		;
		//HAL_Delay(1000);
		lora_busy = 0;
		State = TX;
		UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
				CFG_SEQ_Prio_0);
		break;

	default:
		State = TX;
		UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
				CFG_SEQ_Prio_0);
		break;
	}

}

static void OnledEvent(void *context) {
	HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin); /* LED_GREEN */
	HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin); /* LED_RED */
	//UTIL_TIMER_Start(&timerLed);
}

/* USER CODE END PrFD */
