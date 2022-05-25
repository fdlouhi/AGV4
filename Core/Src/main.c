/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Funciones.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PWM_CENTRO 67

typedef enum
{
	Lectura,
	calculo,
} LecBateria_t;

typedef enum
{
	PulsoON,
	PulsoFalling,
} Tacometro_t;

typedef enum
{
	Apagado,
	Avanzando,
	Dobla_Derecha,
	Dobla_Izquierda,
	Falla,
	Standby,
} Estado_t;

typedef enum
{
	BottonUp,
	BottonFalling,
	BottonDown,
	BottonRising,
} BottonState_t;

typedef enum
{
	ON,
	OFF,
} Encendido_t;

typedef struct
{
	uint16_t PIN;
	GPIO_TypeDef * Port;
	BottonState_t State;
	Tacometro_t State1;
	Encendido_t State2;
	BottonState_t State_ant;
}Entrada_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t adc[1]; /*Utilizado por el ADC*/
float Tension=0; /*Valores de tension instantaneos*/
float Tension_muestras =0; /*Suma las muestras de tension*/
float TensionAve =6; /*Tension promedio*/
int i=0; /*cuenta la cantidad de muestras a tomar del ADC para la tension de bateria*/
int cuento_20ms=0; /*para esperar los 20 ms de los botones*/
int cuento_1s=0;
int cuento_5ms=0;
int Pulsos=0; /*cuenta la cantidad de pulsos cada 1 seg*/
int Pulso_ant=0; /*estado del pulso anterior para detectar el flanco ascendente*/
int velocidad=0;
int VEL=0;	/*variable para el PWM del motor, actualiza la velocidad del AGV*/
int DIR=67; /*variable para el PWM del servomotor, indica cuanto debe doblar tiene que estar en (67) +- 15 */
int cuento_10s=0; /*para realizar la medición de la bateria cada 10 seg*/
int cuento_20ms2=0; /*para tomar muestras de tension de bateria cada 20 ms*/
int blinkingfalla=0; /*Variable para realizar el destello del led de falla*/
int delay_dobla=0; /*Variable utilizada para que doble de a poco el servomotor*/
int esperaStandby=0;/*Variable para esperar los 5 seg para cargar los materiales a transportar*/

LecBateria_t LecBateria=Lectura;

Estado_t Estado=Apagado;

Entrada_t  Velocimetro=
{
	.PIN = GPIO_PIN_12,
	.Port = GPIOB,
	.State = BottonUp,
	.State1 = PulsoON,
	.State2 = OFF,
	.State_ant = BottonUp,
};

Entrada_t  ON_OFF=
{
	.PIN = GPIO_PIN_13,
	.Port = GPIOB,
	.State = BottonUp,
	.State1 = PulsoON,
	.State2 = OFF,
	.State_ant = BottonUp,
};

Entrada_t  Sensor_derecha=
{
	.PIN = GPIO_PIN_14,
	.Port = GPIOB,
	.State = BottonUp,
	.State1 = PulsoON,
	.State2 = OFF,
	.State_ant = BottonUp,
};

Entrada_t  Sensor_izquierda=
{
	.PIN = GPIO_PIN_15,
	.Port = GPIOB,
	.State = BottonUp,
	.State1 = PulsoON,
	.State2 = OFF,
	.State_ant = BottonUp,
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Lee en forma generica la entrada del micro tomando la info del Puerto y del Pin, Devuelve si esta pulsado o no
int readPin (Entrada_t *Entrada)
{
	return HAL_GPIO_ReadPin(Entrada->Port, Entrada->PIN);
}
/*Funcion que actualiza el estado State de los botones confirmando el estado despues de 20 ms y actualiza el estado 2
 para formar un swich ON OFF*/
void updateButton(Entrada_t* Entrada)
{
	switch (Entrada->State)
		{
		case BottonUp:
			  		  if(readPin(Entrada) ==0)
			  		  {
			  			Entrada->State= BottonFalling;
			  			cuento_20ms=0;
			  		  }
			  		  if(Entrada->State_ant==BottonDown)
			  		  {
			  			Entrada->State_ant= BottonUp;
			  			if (Entrada->State2==OFF)
			  				{
			  				Entrada->State2= ON;
			  				}
			  			else
			  				{
			  				Entrada->State2= OFF;
			  				}
			  		  }
		break;

		case BottonFalling:
			  		  if (cuento_20ms==4)
			  		  {
				  			if(readPin(Entrada) == 0)
				  				{
				  				Entrada->State= BottonDown;
				  				}
			  				else
			  				 	 {
			  					 Entrada->State= BottonUp;
			  				 	 }
			  		  }
		break;

		case BottonDown:
			  		 if (readPin(Entrada) == 1)
			  		 {
			  			 	 Entrada->State= BottonRising;
			  			  	 cuento_20ms=0;
			  		 }
			  		 Entrada->State_ant= BottonDown;
		break;

		case BottonRising:
					if (cuento_20ms==4)
			  		{
			  			  		if(readPin(Entrada)==1)
			  			  		{
			  			  			Entrada->State= BottonUp;

			  			  		}
			  			  		else
			  			  		{
			  			  			Entrada->State= BottonDown;
			  			  		}
			  		}
		break;
		}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);//PWM Direccion
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);//PWM Velocidad
  HAL_TIM_Base_Start_IT(&htim2); //Interrupcion Timer cada 5 ms
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/* Actualizo el estado de los botones con la funcion updateButton */
	 updateButton(&ON_OFF) ;
	 updateButton(&Sensor_derecha) ;
	 updateButton(&Sensor_izquierda) ;

	 /*Calculo de Velocidad*/
	 /*Cada 1 seg calcula cantidad de pulsos y realiza la converción a la valriable velocidad
	 *El tacometro funciona tomando muestras y cada vez que detecta un flanco ascendente incrementa
	 *la cantidad de pulsos, al transcurrir 1 seg realizar el calculo de velocidad y resetea las variables.
	 *El periodo máxima del tacómetro es de 20ms y se decide tomar una muestra de 5 ms porque el Duty Cicle
	 *no es 50%, el tiempo en off es la mitad del tiempo en ON
	 *las entradas estan en resistencia pull up, 0 es activado y 1 desactivado*/

	 if (cuento_1s==200)/*Espera que transcurra un 1 seg y resetea variables*/
	 {
		 cuento_1s=0;
		 velocidad=Pulsos*0.212*60/20;	//calcula velocidad en metros/minutos
		 Pulsos=0;
	 }
	 else
	 {
		 if (cuento_5ms==1) /*cada 5 ms tomo una muestra y actualizo el estado State1*/
		 {
			cuento_5ms=0;
			switch (Velocimetro.State1)
			{
				case PulsoON:
				Pulso_ant=1;
				if (readPin(&Velocimetro)==1)
						{
						Velocimetro.State1= PulsoFalling;
						}
				break;

				case PulsoFalling:
				if (Pulso_ant==1)
					{
					Pulsos++;
					Pulso_ant=0;
					}
				if (readPin(&Velocimetro)==0)
					{
					Velocimetro.State1= PulsoON;
					}
				break;
			}
		 }
	 }


  /*Indica el estado del AGV, En el caso de  que el promedio de tension baje de 3.5 entra en falla y no sale de ahi
   Cuando se encuentra en apagado centra las ruedas y apaga baja la velocidad a 0, en encendido dependiendo de la
   posición de las ruedas aumenta o baja la velocidad, en los estado de doblar solamente ajusta la direccion
   y vuelve al estado avanzar.*/

  switch (Estado)
  	  {
  	  case Apagado:
  		if (TensionAve <= 1.0)
  		  		{
  		  		Estado=Falla;
  		  	    blinkingfalla=0;
  		  		}
  		  VEL=0;
  		  DIR=PWM_CENTRO;
  		  if (ON_OFF.State2 == ON)
  		  	  {
  			   Estado=Avanzando;
  			  }
  		  break;
  	  case Avanzando:

  		  if (TensionAve <= 1.0)
  		  		  		{
  		  		  		Estado=Falla;
  		  		  	    blinkingfalla=0;
  		  		  		}
  		if(DIR>=70 || DIR<=64)
  			{
  			 VEL=300;
  			}
  		else
  			{
  			 VEL=1000;
  			}
  		  if (ON_OFF.State2 == OFF)
  		  	  {
  			  Estado=Apagado;
  			  }
  		 else
  		 	 {
  			if (Sensor_izquierda.State == BottonDown)
  			  		  	  {
  			  		  	  Estado=Dobla_Izquierda;
  			  		      delay_dobla=0;
  			  		  	  }
  			if (Sensor_derecha.State == BottonDown)
  			  			  {
  			  			  Estado=Dobla_Derecha;
  			  			  delay_dobla=0;
  			  			  }
  		 	 }

  		  break;
  	  case Dobla_Derecha:
  		if (TensionAve <= 1.0)
  		  	{
  		  	Estado=Falla;
  		    blinkingfalla=0;
  		  	}
  		  if (ON_OFF.State2 == OFF)
  		  	  {
  			  Estado=Apagado;
  			  }
  		else
  			  {
  				if(delay_dobla>=50)
  				{
  					delay_dobla=0;
  				if(DIR<82)
  					{
  					DIR=DIR+5;
  					}
  				Estado=Avanzando;
  				}
  				else
  				{
  				if (Sensor_izquierda.State == BottonDown)
  				  			  		  	  {
  				  			  		  	  Estado=Standby;
  				  			  		esperaStandby=0;
  				  			  		      }
  				}
  				}

	  	  break;
  	  case Dobla_Izquierda:
  		if (TensionAve <= 1.0)
  		  	{
  		  	Estado=Falla;
  		    blinkingfalla=0;
  		  	}
  		  if (ON_OFF.State2 == OFF)
  		  	  {
  			  Estado=Apagado;
  			  }
  		else
  			{
  			if(delay_dobla>=50)
  			{
  			delay_dobla=0;
  			if(DIR>52)
  				{
  				 DIR=DIR-5;
  				}
  			Estado=Avanzando;
  			}
  			else
  			  	{
  			  	if (Sensor_derecha.State == BottonDown)
  			  				  		  	  {
  			  	  			  		  	  Estado=Standby;
  			  	  			  		  	  esperaStandby=0;
  			  	  			  		      }
  			  	}

  			}
  		  break;
  	  case Falla:

  		  if (blinkingfalla==100)
  		  {
  			VEL=0;
  			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
  			blinkingfalla=0;

  		  }
  		  break;
  	  case Standby:
  		  VEL=0;
  		  if(esperaStandby>=1000)
  		  {
  			Estado=Avanzando;
  		  }
  		  break;
  	 }

  /*Lectura cada 10 seg de la tension de bateria, toma 10 muetras cada 20 ms y realiza un promedio*/

  	  if (cuento_10s ==2000)
      {

	  LecBateria=Lectura;
	  cuento_20ms2=0;
	  i=0;
	  Tension_muestras=0;
	  cuento_10s=0;

      }
	  switch (LecBateria)
  	  {
  	  case  Lectura:

  		  if (cuento_20ms2==4)
  		  {
  		  HAL_ADC_Start(&hadc1); /*Enciendo el ADC tomo la muestra y lo apago*/
  		  HAL_ADC_PollForConversion(&hadc1, 10);
  		  adc[0]=HAL_ADC_GetValue(&hadc1);
  		  HAL_ADC_Stop(&hadc1);
  	      Tension=adc[0]*0.0014;
  	      Tension_muestras += Tension;
  		  i++;
  		  cuento_20ms2=0;
  		  }
  		  if (i==9)
  		  {
  		  LecBateria=calculo;
  		  }
  		  break;
  	  case 	calculo:

  		 TensionAve = Tension_muestras/(i);

  		 break;
  	  }


  htim1.Instance->CCR1=DIR; /*Inicializo la direccion derecho (67) +- 15*/
  htim1.Instance->CCR2=VEL; /*Inicializo Velocidad 0*/
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1440;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 720;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : AV2_Pin AV1_Pin VEL_Pin ON_OFF_Pin
                           IN_2_Pin IN_1_Pin */
  GPIO_InitStruct.Pin = AV2_Pin|AV1_Pin|VEL_Pin|ON_OFF_Pin
                          |IN_2_Pin|IN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
cuento_20ms++;
cuento_1s++;
cuento_5ms++;
cuento_10s++;
cuento_20ms2++;
blinkingfalla++;
delay_dobla++;
esperaStandby++;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
