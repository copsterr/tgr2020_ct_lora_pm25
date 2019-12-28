/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @brief   this is the main!
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "low_power_manager.h"
#include "lora.h"
#include "bsp.h"
#include "timeServer.h"
#include "vcom.h"
#include "version.h"

#include "ct_honey.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define LORAWAN_MAX_BAT   254

/*!
 * CAYENNE_LPP is myDevices Application server.
 */
//#define CAYENNE_LPP
#define LPP_DATATYPE_DIGITAL_INPUT  0x0
#define LPP_DATATYPE_DIGITAL_OUTPUT 0x1
#define LPP_DATATYPE_HUMIDITY       0x68
#define LPP_DATATYPE_TEMPERATURE    0x67
#define LPP_DATATYPE_BAROMETER      0x73
#define LPP_APP_PORT 99
/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            60000
#define SETTING_MODE_DUTYCYCLE						5000
/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE LORAWAN_ADR_ON
/*!
 * LoRaWAN Default data Rate Data Rate
 * @note Please note that LORAWAN_DEFAULT_DATA_RATE is used only when ADR is disabled
 */
#define LORAWAN_DEFAULT_DATA_RATE DR_0
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            2
/*!
 * LoRaWAN default endNode class port
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_A
/*!
 * LoRaWAN default confirm state
 */
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE           LORAWAN_UNCONFIRMED_MSG
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE                           64
/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];

/*!
 * User application data structure
 */
//static lora_AppData_t AppData={ AppDataBuff,  0 ,0 };
lora_AppData_t AppData = { AppDataBuff,  0, 0 };

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa endNode has received a frame*/
static void LORA_RxData(lora_AppData_t *AppData);

/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined(void);

/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass(DeviceClass_t Class);

/* call back when server needs endNode to send a frame*/
static void LORA_TxNeeded(void);

/* callback to get the battery level in % of full charge (254 full charge, 0 no charge)*/
static uint8_t LORA_GetBatteryLevel(void);

/* LoRa endNode send request*/
static void Send(void *context);

/* start the tx process*/
static void LoraStartTx(TxEventType_t EventType);

/* tx timer callback function*/
static void OnTxTimerEvent(void *context);

/* tx timer callback function*/
static void LoraMacProcessNotify(void);

/* Private variables ---------------------------------------------------------*/
/* load Main call backs structure*/
static LoRaMainCallback_t LoRaMainCallbacks = { LORA_GetBatteryLevel,
                                                HW_GetTemperatureLevel,
                                                HW_GetUniqueId,
                                                HW_GetRandomSeed,
                                                LORA_RxData,
                                                LORA_HasJoined,
                                                LORA_ConfirmClass,
                                                LORA_TxNeeded,
                                                LoraMacProcessNotify
                                              };
LoraFlagStatus LoraMacProcessRequest = LORA_RESET;
LoraFlagStatus AppProcessRequest = LORA_RESET;
/*!
 * Specifies the state of the application LED
 */
static uint8_t AppLedStateOn = RESET;

static TimerEvent_t TxTimer;
static TimerEvent_t SettingTimer;

#ifdef USE_B_L072Z_LRWAN1
/*!
 * Timer to handle the application Tx Led to toggle
 */
static TimerEvent_t TxLedTimer;
static void OnTimerLedEvent(void *context);
#endif
/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit = {LORAWAN_ADR_STATE,
                                     LORAWAN_DEFAULT_DATA_RATE,
                                     LORAWAN_PUBLIC_NETWORK
                                    };

/* Private functions ---------------------------------------------------------*/
static void initUserBtn(void);
static void initUart1(void);
static void initTim6(void);

static void OnSettingModeElapsed(void *context);
static void StartSettingModeElapsed();


/* Private Vars --------------------------------------------------------------*/
UART_HandleTypeDef huart1;
TIM_HandleTypeDef  htim6;

//#define SEND_GEO_DATA // for sending only geo data
#define DATA_TOGGLING // for toggling data sending between pm2.5 and geolocation
uint8_t send_pm_toggler = 1;

// mode control
volatile uint8_t setting_mode = 1;
volatile uint8_t setting_mode_timeout_count = 0;
#define SETTING_MODE_TIMEOUT_COUNT_MAX 25

// honey vars
#define HONEY_WARMUP_DURATION 10000
honey_t honey;


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{


  /* STM32 HAL library initialization*/
  HAL_Init();

  /* Configure the system clock*/
  SystemClock_Config();


  /* Configure the debug mode*/
  DBG_Init();

  /* Configure the hardware*/
  HW_Init();

  // Init connectivity peripherals
  initTim6();
  initUart1();
  initUserBtn();

  /* USER CODE BEGIN 1 */
  volatile uint32_t time = 0;
  volatile uint32_t time_tick = 0;
  volatile uint32_t time_ms = 0;

  // init pm2.5 module
  honey_init(huart1, &honey);


  /* USER CODE END 1 */

  /*Disbale Stand-by mode*/
  LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);

  PRINTF("APP_VERSION= %02X.%02X.%02X.%02X\r\n", (uint8_t)(__APP_VERSION >> 24), (uint8_t)(__APP_VERSION >> 16), (uint8_t)(__APP_VERSION >> 8), (uint8_t)__APP_VERSION);
  PRINTF("MAC_VERSION= %02X.%02X.%02X.%02X\r\n", (uint8_t)(__LORA_MAC_VERSION >> 24), (uint8_t)(__LORA_MAC_VERSION >> 16), (uint8_t)(__LORA_MAC_VERSION >> 8), (uint8_t)__LORA_MAC_VERSION);

  /* Configure the Lora Stack*/
  LORA_Init(&LoRaMainCallbacks, &LoRaParamInit);

  LORA_Join();

//  LoraStartTx(TX_ON_TIMER);

  // LOOP
  while (1)
  {
	if (!setting_mode) {
		// polling for user to press userbtn input
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == 0) {
			HAL_Delay(1000); // debouncing prevention

			// stop txtimer and start elapsing timer
			TimerStop(&TxTimer);
			StartSettingModeElapsed();

			setting_mode = 1; // change mode

			PRINTF("\r\n[i] TxTimer Stopped\r\n[i] Entering Setting Mode...\r\n\r\n");
		}

	/* Normal Mode Start ---------------------------------------------------- */
		if (AppProcessRequest == LORA_SET)
		{
		  /*reset notification flag*/
		  AppProcessRequest = LORA_RESET;
		  /*Send*/
		  PRINTF("STARTING UP PM2.5 MEASUREMENT...\r\n");
		  honey_start(&honey);
		  HAL_Delay(HONEY_WARMUP_DURATION);

		  PRINTF("Transmitting PM2.5 Concentration...\r\n");
		  Send(NULL);

		  honey_stop(&honey);
		  PRINTF("TRANSMISSION COMPLETED.\r\n");
		}
		if (LoraMacProcessRequest == LORA_SET)
		{
		  /*reset notification xcflag*/
		  LoraMacProcessRequest = LORA_RESET;
		  LoRaMacProcess();
		}
		/*If a flag is set at this point, mcu must not enter low power and must loop*/
		DISABLE_IRQ();

	    /* if an interrupt has occurred after DISABLE_IRQ, it is kept pending
	     * and cortex will not enter low power anyway  */
		if ((LoraMacProcessRequest != LORA_SET) && (AppProcessRequest != LORA_SET))
		{
#ifndef LOW_POWER_DISABLE
    LPM_EnterLowPower();
#endif
	    }

	    ENABLE_IRQ();
	/* Normal Mode End ------------------------------------------------------ */
	}
	else {
	/* Setting Mode Start --------------------------------------------------- */
		// timeout control
		if (setting_mode_timeout_count == SETTING_MODE_TIMEOUT_COUNT_MAX) {
			PRINTF("\r\n[i] SETTING MODE TIMEOUT, entering normal mode...");
			TimerStop(&SettingTimer); // stop setting mode timer
			TimerReset(&SettingTimer);
			setting_mode = 0; // change mode to normal
			LoraStartTx(TX_ON_TIMER);
		}


	/* Setting Mode End ----------------------------------------------------- */
	}

  }
}


void LoraMacProcessNotify(void)
{
  LoraMacProcessRequest = LORA_SET;
}


static void LORA_HasJoined(void)
{
#if( OVER_THE_AIR_ACTIVATION != 0 )
  PRINTF("JOINED\n\r");
#endif
  LORA_RequestClass(LORAWAN_DEFAULT_CLASS);
}

static void Send(void *context)
{
  /* USER CODE BEGIN 3 */
//  uint16_t pressure;
//  int16_t temperature;
//  uint16_t humidity;
//  sensor_t sensor_data;
  uint8_t  batteryLevel;
  uint16_t pm2_5;			// pm2.5 concentration
  uint8_t  sensor_err = 0;  // if sensor has error
  uint8_t  lowBatt = 0; 	// Indicates that battery is low

  if (LORA_JoinStatus() != LORA_SET)
  {
    /*Not joined, try again later*/
    LORA_Join();
    return;
  }

  TVL1(PRINTF("SEND REQUEST\n\r");)
#ifndef CAYENNE_LPP
  int32_t latitude, longitude = 0;
//  uint16_t altitudeGps = 0;
#endif

#ifdef USE_B_L072Z_LRWAN1
  TimerInit(&TxLedTimer, OnTimerLedEvent);

  TimerSetValue(&TxLedTimer, 200);

  LED_On(LED_RED1) ;

  TimerStart(&TxLedTimer);
#endif

//  BSP_sensor_Read(&sensor_data);

  // read pm2.5 from Honeywell sensor
  if (honey_read(&honey) == CMD_RESP_SUCCESS) {
	  PRINTF("[s] Read PM2.5 Success!\r\n");
	  pm2_5 = honey.pm2_5;
	  if (pm2_5 == 191) {
		  pm2_5 = 190;
	  }
  } else {
	  PRINTF("[e] Read PM2.5 Error!\r\n");
	  sensor_err = 1;
  }

#ifdef CAYENNE_LPP
  uint8_t cchannel = 0;
  temperature = (int16_t)(sensor_data.temperature * 10);         /* in �C * 10 */
  pressure    = (uint16_t)(sensor_data.pressure * 100 / 10);      /* in hPa / 10 */
  humidity    = (uint16_t)(sensor_data.humidity * 2);            /* in %*2     */
  uint32_t i = 0;

  batteryLevel = LORA_GetBatteryLevel();                      /* 1 (very low) to 254 (fully charged) */

  AppData.Port = LPP_APP_PORT;

  AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_BAROMETER;
  AppData.Buff[i++] = (pressure >> 8) & 0xFF;
  AppData.Buff[i++] = pressure & 0xFF;
  AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_TEMPERATURE;
  AppData.Buff[i++] = (temperature >> 8) & 0xFF;
  AppData.Buff[i++] = temperature & 0xFF;
  AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_HUMIDITY;
  AppData.Buff[i++] = humidity & 0xFF;
#if defined( REGION_US915 ) || defined ( REGION_AU915 ) || defined ( REGION_AS923 )
  /* The maximum payload size does not allow to send more data for lowest DRs */
#else
  AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_DIGITAL_INPUT;
  AppData.Buff[i++] = batteryLevel * 100 / 254;
  AppData.Buff[i++] = cchannel++;
  AppData.Buff[i++] = LPP_DATATYPE_DIGITAL_OUTPUT;
  AppData.Buff[i++] = AppLedStateOn;
#endif  /* REGION_XX915 */
#else  /* not CAYENNE_LPP */


//  temperature = (int16_t)(sensor_data.temperature * 100);         /* in �C * 100 */
//  pressure    = (uint16_t)(sensor_data.pressure * 100 / 10);      /* in hPa / 10 */
//  humidity    = (uint16_t)(sensor_data.humidity * 10);            /* in %*10     */
//  latitude = sensor_data.latitude;
//  longitude = sensor_data.longitude;

//lat 13.73654, long 100.52877 of CU Engineering
  latitude = 1373654;
  longitude = 10052877;
  uint32_t i = 0;

  // check battery level
  batteryLevel = LORA_GetBatteryLevel();                      /* 1 (very low) to 254 (fully charged) */
  if (batteryLevel < 5) {
    lowBatt = 1;
  }

  AppData.Port = LORAWAN_APP_PORT;

#if defined( REGION_US915 ) || defined ( REGION_AU915 ) || defined ( REGION_AS923 )
  if (lowBatt || sensor_err) {
	// if sensor is error or battery is low send 191
	AppData.Buff[i++] = 17;
	AppData.Buff[i++] = 191;
  } else {

#ifdef DATA_TOGGLING
	if (send_pm_toggler) {
		PRINTF("[i] sending pm2.5 data...\r\n");
		AppData.Buff[i++] = 17;
		AppData.Buff[i++] = pm2_5;
	} else {
		PRINTF("[i] sending geolocation data...\r\n");
		AppData.Buff[i++] = 17;
		AppData.Buff[i++] = (latitude >> 16) & 0xFF;
		AppData.Buff[i++] = (latitude >> 8) & 0xFF;
		AppData.Buff[i++] = latitude & 0xFF;
		AppData.Buff[i++] = (longitude >> 16) & 0xFF;
		AppData.Buff[i++] = (longitude >> 8) & 0xFF;
		AppData.Buff[i++] = longitude & 0xFF;
	}
	send_pm_toggler = !send_pm_toggler;
#else
  	PRINTF("[i] sending pm2.5 data...\r\n");
  	AppData.Buff[i++] = 17;
  	AppData.Buff[i++] = pm2_5;
#endif
}

#else  /* not REGION_XX915 */
  // AppData.Buff[i++] = AppLedStateOn;
  // AppData.Buff[i++] = (pressure >> 8) & 0xFF;
  // AppData.Buff[i++] = pressure & 0xFF;
  if (lowBatt) {
    AppData.Buff[i++] = 191;
  } else {
    AppData.Buff[i++] = (temperature >> 8) & 0xFF;
    AppData.Buff[i++] = temperature & 0xFF;
    AppData.Buff[i++] = (humidity >> 8) & 0xFF;
    AppData.Buff[i++] = humidity & 0xFF;
    AppData.Buff[i++] = batteryLevel;
    AppData.Buff[i++] = (latitude >> 16) & 0xFF;
    AppData.Buff[i++] = (latitude >> 8) & 0xFF;
    AppData.Buff[i++] = latitude & 0xFF;
    AppData.Buff[i++] = (longitude >> 16) & 0xFF;
    AppData.Buff[i++] = (longitude >> 8) & 0xFF;
    AppData.Buff[i++] = longitude & 0xFF;
  }

#endif  /* REGION_XX915 */
#endif  /* CAYENNE_LPP */
  AppData.BuffSize = i;

  LORA_send(&AppData, LORAWAN_DEFAULT_CONFIRM_MSG_STATE);

  /* USER CODE END 3 */
}


static void LORA_RxData(lora_AppData_t *AppData)
{
  /* USER CODE BEGIN 4 */
  PRINTF("PACKET RECEIVED ON PORT %d\n\r", AppData->Port);

  switch (AppData->Port)
  {
    case 3:
      /*this port switches the class*/
      if (AppData->BuffSize == 1)
      {
        switch (AppData->Buff[0])
        {
          case 0:
          {
            LORA_RequestClass(CLASS_A);
            break;
          }
          case 1:
          {
            LORA_RequestClass(CLASS_B);
            break;
          }
          case 2:
          {
            LORA_RequestClass(CLASS_C);
            break;
          }
          default:
            break;
        }
      }
      break;
    case LORAWAN_APP_PORT:
      if (AppData->BuffSize == 1)
      {
        AppLedStateOn = AppData->Buff[0] & 0x01;
        if (AppLedStateOn == RESET)
        {
          PRINTF("LED OFF\n\r");
          LED_Off(LED_BLUE) ;
        }
        else
        {
          PRINTF("LED ON\n\r");
          LED_On(LED_BLUE) ;
        }
      }
      break;
    case LPP_APP_PORT:
    {
      AppLedStateOn = (AppData->Buff[2] == 100) ?  0x01 : 0x00;
      if (AppLedStateOn == RESET)
      {
        PRINTF("LED OFF\n\r");
        LED_Off(LED_BLUE) ;

      }
      else
      {
        PRINTF("LED ON\n\r");
        LED_On(LED_BLUE) ;
      }
      break;
    }
    default:
      break;
  }
  /* USER CODE END 4 */
}


static void OnSettingModeElapsed(void *context)
{
	TimerStart(&SettingTimer);
	setting_mode_timeout_count++; // increment timeout counter
}

static void StartSettingModeElapsed()
{
	TimerInit(&SettingTimer, OnSettingModeElapsed);
	TimerSetValue(&SettingTimer,  SETTING_MODE_DUTYCYCLE);
	OnSettingModeElapsed(NULL);
}

static void OnTxTimerEvent(void *context)
{
  /*Wait for next tx slot*/
  TimerStart(&TxTimer);

  AppProcessRequest = LORA_SET;
}

static void LoraStartTx(TxEventType_t EventType)
{
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */
    TimerInit(&TxTimer, OnTxTimerEvent);
    TimerSetValue(&TxTimer,  APP_TX_DUTYCYCLE);
    OnTxTimerEvent(NULL);
  }
  else
  {
    /* send everytime button is pushed */
    GPIO_InitTypeDef initStruct = {0};

    initStruct.Mode = GPIO_MODE_IT_RISING;
    initStruct.Pull = GPIO_PULLUP;
    initStruct.Speed = GPIO_SPEED_HIGH;

    HW_GPIO_Init(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, &initStruct);
    HW_GPIO_SetIrq(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 0, Send);
  }
}

static void LORA_ConfirmClass(DeviceClass_t Class)
{
  PRINTF("switch to class %c done\n\r", "ABC"[Class]);

  /*Optionnal*/
  /*informs the server that switch has occurred ASAP*/
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;

  LORA_send(&AppData, LORAWAN_UNCONFIRMED_MSG);
}

static void LORA_TxNeeded(void)
{
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;

  LORA_send(&AppData, LORAWAN_UNCONFIRMED_MSG);
}

/**
  * @brief This function return the battery level
  * @param none
  * @retval the battery level  1 (very low) to 254 (fully charged)
  */
uint8_t LORA_GetBatteryLevel(void)
{
  uint16_t batteryLevelmV;
  uint8_t batteryLevel = 0;

  batteryLevelmV = HW_GetBatteryLevel();


  /* Convert batterey level from mV to linea scale: 1 (very low) to 254 (fully charged) */
  if (batteryLevelmV > VDD_BAT)
  {
    batteryLevel = LORAWAN_MAX_BAT;
  }
  else if (batteryLevelmV < VDD_MIN)
  {
    batteryLevel = 0;
  }
  else
  {
    batteryLevel = (((uint32_t)(batteryLevelmV - VDD_MIN) * LORAWAN_MAX_BAT) / (VDD_BAT - VDD_MIN));
  }

  return batteryLevel;
}

#ifdef USE_B_L072Z_LRWAN1
static void OnTimerLedEvent(void *context)
{
  LED_Off(LED_RED1) ;
}
#endif

static void initUserBtn(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//	HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
//	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
//	HW_GPIO_Init(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, &GPIO_InitStruct);
//	HW_GPIO_SetIrq(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 0, printStuff);
}

static void initUart1(void)
{
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
}

static void initTim6(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 47999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM6)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM6_CLK_ENABLE();
    /* TIM6 interrupt Init */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  }
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
