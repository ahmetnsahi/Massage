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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Programs.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

int a1,a2,a3;
 uint16_t tempduty=0,motorduty=0;
uint16_t timerx=0,alarmt=0,timer,sa;
uint16_t total_time=0;
uint8_t start=0,saniye,dakika,saat;
extern stopf;


Control ms;
Control mst;

uint16_t isi=0;
uint16_t hız=0;
uint16_t darbes=0;
uint16_t darber=0;
uint8_t pFlag=0;
uint8_t delay1=0;
uint8_t delay2=0;

#define RxBuffer_Size1 10
#define RxBuffer_Size 60
uint8_t RxBuf [RxBuffer_Size];
uint8_t RxBuf1 [RxBuffer_Size1];

/* USER CODE END PTD */
char str[80];
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1,HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){

 if (huart-> Instance==USART2) {

		  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuffer_Size);
		  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT); //Buffer alınırken yarıda kesmeye gitmesin diyeyapılmış işlemi.

		    for (int i  = Size;  i<RxBuffer_Size; i++ ) { //BUFFER ESKİLERİ SİLME
		  				   			  RxBuf[i]=0;	}
 }


 if (RxBuf[0]=='M' && RxBuf[1]=='D'){
	 selectmotorduty();
	 RxBuf[0]=0;
	 RxBuf[1]=0;
	 RxBuf[2]=0;
	HAL_UART_Transmit_DMA(&huart2, "OK\n\r", 4);
	printf("Motor Speed=%d",motorduty);


 }

if (RxBuf[0]=='T' && RxBuf[1]=='D'){
		 selecttempduty();
		  RxBuf[0]=0;
		  RxBuf[1]=0;
		  RxBuf[2]=0;
		HAL_UART_Transmit_DMA(&huart2, "OK\n\r", 4);
		printf("Motor Speed=%d",tempduty);
}




}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==htim6.Instance){
		timerx++;


     if(start==1)
	    {
    	 pFlag=0;
	      timer--;

		    if(timer==0)
			    {
			              timer=0 ;
			              start=0;
			    }
		    printf("Kalan süre= %d\n", timer);
	    }

     if(pFlag==1)
	    {
    	  start=2;
	      timer--;

		    if(timer==0)
			    {
			              timer=0;
			              pFlag=0;
			    }
		    sprintf(str, "Kalan süre= %d\n", timer);
		    HAL_UART_Transmit_DMA(&huart2, str, strlen(str));

	    }





    }

}


void programselected(){


	 // motor seçimi
	 uint16_t amask=0b1110000000000000;
	 uint16_t amast=0b0001111111111111;
	 a1 =RxBuf[2];
	 a1=a1<<8;
	 a1=a1+RxBuf[3];
	 ms.MotorSayi=a1 & amast;  //sadece motor kontroll değişkneleri
	 mst.MotorSayi=a1 & amask; //sadece ısıtıcı değişkenleri


	  if(RxBuf[4]==0){
		  a2=0;
	  }
	  else if(RxBuf[4]==1) {
		  a2=500;
	  } else if(RxBuf[4]==2) {
		  a2=550;
	  } else if(RxBuf[4]==3) {
		  a2=600;
	  } else if(RxBuf[4]==4) {
		  a2=700;
	  } else if(RxBuf[4]==5) {
		  a2=750;
	  }   else if(RxBuf[4]==6) {
		  a2=800;
	  }    else if(RxBuf[4]==6) {
		  a2=850;
	  }    else if(RxBuf[4]==8) {
		  a2=900;
	  }   else if(RxBuf[4]==9) {
		  a2=950;
	  }    else if(RxBuf[4]==10) {
		  a2=1000;
	  }
	  else  ;


	  if(RxBuf[5]==0){
		  a3=0;
	  }
	  else if(RxBuf[5]==1) {
		  a3=500;
	  } else if(RxBuf[5]==2) {
		  a3=550;
	  } else if(RxBuf[5]==3) {
		  a3=600;
	  } else if(RxBuf[5]==4) {
		  a3=700;
	  } else if(RxBuf[5]==5) {
		  a3=750;
	  }   else if(RxBuf[5]==6) {
		  a3=800;
	  }    else if(RxBuf[5]==6) {
		  a3=850;
	  }    else if(RxBuf[5]==8) {
		  a3=900;
	  }   else if(RxBuf[5]==9) {
		  a3=950;
	  }    else if(RxBuf[5]==10) {
		  a3=1000;
	  }
	  else  ;

	  //------------------------------------ Delayed Seçimileri---------------------------------/
	  if(RxBuf[7]==0){
		 delay1=0;
		 delay2=0;
	  }
	  else if(RxBuf[7]==1) {
			 delay1=1;
			 delay2=1;
	  } else if(RxBuf[7]==2) {
			 delay1=2;
			 delay2=2;
	  } else if(RxBuf[7]==3) {
			 delay1=1;
			 delay2=4;
	  } else if(RxBuf[7]==4) {
			 delay1=1;
			 delay2=3;
	  } else if(RxBuf[7]==5) {
			 delay1=1;
			 delay2=2;
	  }   else if(RxBuf[7]==6) {
			 delay1=0;
			 delay2=0;
	  }    else if(RxBuf[7]==6) {
			 delay1=0;
			 delay2=0;
	  }    else if(RxBuf[7]==8) {
			 delay1=0;
			 delay2=0;
	  }   else if(RxBuf[7]==9) {
			 delay1=0;
			 delay2=0;
	  }    else if(RxBuf[7]==10) {
			 delay1=0;
			 delay2=0;
	  }
	  else  ;



	 ms.MotorDuty=a2;
	 mst.TempDuty=a3;
	 Pwm_Start(mst);
     Pwm_Start(ms);





}

void standarprgslctd(){


		  if(RxBuf[8]==0){                 //motorlar için duty seçimi
			  motorduty=0;
		  }
		  else if(RxBuf[8]==1) {
			  motorduty=500;
		  } else if(RxBuf[8]==2) {
			  motorduty=550;
		  } else if(RxBuf[8]==3) {
			  motorduty=600;
		  } else if(RxBuf[8]==4) {
			  motorduty=700;
		  } else if(RxBuf[8]==5) {
			  motorduty=750;
		  } else if(RxBuf[8]==6) {
			  motorduty=800;
		  } else if(RxBuf[8]==7) {
			  motorduty=850;
		  } else if(RxBuf[8]==8) {
			  motorduty=900;
		  } else if(RxBuf[8]==9) {
			  motorduty=950;
		  } else if(RxBuf[8]==10) {
			  motorduty=1000;
		  }
		  else {
			  motorduty=500;
		  }


		  if(RxBuf[9]==0){                  //Isıstıcı için duty seçimi
		 				  tempduty=0;
		 			  }
		 			    else if(RxBuf[9]==1) {
		 				  tempduty=500;
		 			  } else if(RxBuf[9]==2) {
		 				  tempduty=550;
		 			  } else if(RxBuf[9]==3) {
		 				  tempduty=600;
		 			  } else if(RxBuf[9]==4) {
		 				  tempduty=700;
		 			  } else if(RxBuf[9]==5) {
		 				  tempduty=750;
		 			  } else if(RxBuf[9]==6) {
		 				  tempduty=800;
		 			  } else if(RxBuf[9]==7) {
		 				  tempduty=850;
		 			  } else if(RxBuf[9]==8) {
		 				  tempduty=900;
		 			  } else if(RxBuf[9]==9) {
		 				  tempduty=950;
		 			  } else if(RxBuf[9]==10) {
		 				  tempduty=1000;
		 			  }
		 		   else{
		 			     tempduty=500;
		 			  }




}


void selectmotorduty(){

	  if(RxBuf[2]==0){                 //motorlar için duty seçimi
		  motorduty=0;
	  }
	  else if(RxBuf[2]==1) {
		  motorduty=500;
	  } else if(RxBuf[2]==2) {
		  motorduty=550;
	  } else if(RxBuf[2]==3) {
		  motorduty=600;
	  } else if(RxBuf[2]==4) {
		  motorduty=700;
	  } else if(RxBuf[2]==5) {
		  motorduty=750;
	  } else if(RxBuf[2]==6) {
		  motorduty=800;
	  } else if(RxBuf[2]==7) {
		  motorduty=850;
	  } else if(RxBuf[2]==8) {
		  motorduty=900;
	  } else if(RxBuf[2]==9) {
		  motorduty=950;
	  } else if(RxBuf[2]==10) {
		  motorduty=1000;
	  }
	  else {
		  motorduty=500;
	  }
	  printf("Motor char=%d",RxBuf[2]);

}

void selecttempduty(){
	 if(RxBuf[2]==0){                  //Isıstıcı için duty seçimi
				  tempduty=0;
			  }
			    else if(RxBuf[2]==1) {
				  tempduty=500;
			  } else if(RxBuf[2]==2) {
				  tempduty=550;
			  } else if(RxBuf[2]==3) {
				  tempduty=600;
			  } else if(RxBuf[2]==4) {
				  tempduty=700;
			  } else if(RxBuf[2]==5) {
				  tempduty=750;
			  } else if(RxBuf[2]==6) {
				  tempduty=800;
			  } else if(RxBuf[2]==7) {
				  tempduty=850;
			  } else if(RxBuf[2]==8) {
				  tempduty=900;
			  } else if(RxBuf[2]==9) {
				  tempduty=950;
			  } else if(RxBuf[2]==10) {
				  tempduty=1000;
			  }
		   else{
			     tempduty=500;
			  }


}

void standart(){

	 //dogrulma komutu ve timer sıfırlama...
		  if(RxBuf[0]=='R' && RxBuf[1]=='S'&& RxBuf[2]=='T')
		  { RxBuf[0]=0;
		    RxBuf[1]=0;
		    RxBuf[2]=0;
		    timerx=0;
		    pFlag=0;
		    HAL_UART_Transmit_DMA(&huart2, "OK\n\r", 4);
		  }

		  //Tüm sistemi durdurma...
		  if(RxBuf[0]=='S' && RxBuf[1]=='T'&& RxBuf[2]=='P')
		 	  {
			   stop:
		 	   start=0;
		 	   timer=0;
			   HAL_UART_Transmit_DMA(&huart2, "OK\n\r", 4);
			   a1=0b1111111111111111;  //Tüm sistem pin seçildi
		 	   a2=0;                   //duty sıfırlandı
		 	   ms.MotorSayi=a1;
		 	   ms.MotorDuty=a2;
		 	   ms.TempDuty=a2;
		 	   Pwm_Start(ms);

		 	   pFlag=0;
		 	   timerx=0;


		 	   RxBuf[0]=0;
		 	   RxBuf[1]=0;
		 	   RxBuf[2]=0;

		 	 }

		  //Tüm sistemi aktif test...
		  if(RxBuf[0]=='T' && RxBuf[1]=='S'&& RxBuf[2]=='1')
		 	  {
			    RxBuf[0]=0;
			    RxBuf[1]=0;
			    RxBuf[2]=0;

			   a1=0b1111111111111111;  //Tüm sistem pin seçildi
		 	   a2=700;                   //duty sıfırlandı
		 	   ms.MotorSayi=a1;
		 	   ms.MotorDuty=a2;
		 	   ms.TempDuty=a2;
		 	   Pwm_Start(ms);
		 	   //pFlag=0;
		 	   timerx=0;
		 	   HAL_UART_Transmit_DMA(&huart2, "OK\n\r", 4);

		 	   }
		  //Tüm sistemi aktif test2...
		  if(RxBuf[0]=='T' && RxBuf[1]=='S'&& RxBuf[2]=='2')
		 	  {
			    RxBuf[0]=0;
			    RxBuf[1]=0;
			    RxBuf[2]=0;

			   a1=0b1111111111111111;  //Tüm sistem pin seçildi
		 	   a2=1000;                   //duty sıfırlandı
		 	   ms.MotorSayi=a1;
			   ms.MotorDuty=a2;
			   ms.TempDuty=a2;
		 	   Pwm_Start(ms);
		 	  //pFlag=0;
		 	   timerx=0;
		 	   HAL_UART_Transmit_DMA(&huart2, "OK\n\r", 4);

		 	   }
		  //Tüm sistemi aktif test3...
		  if(RxBuf[0]=='T' && RxBuf[1]=='S'&& RxBuf[2]=='3')
		 	  {
			    RxBuf[0]=0;
			    RxBuf[1]=0;
			    RxBuf[2]=0;

			   a1=0b0001111111111111;  //Tüm sistem pin seçildi
		 	   a2=1000;                   //duty sıfırlandı
		 	   ms.MotorSayi=a1;
			   ms.MotorDuty=a2;
			   ms.TempDuty=a2;
		 	   Pwm_Start(ms);
		 	   //pFlag=0;
		 	   timerx=0;
		 	   HAL_UART_Transmit_DMA(&huart2, "OK\n\r", 4);

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuffer_Size);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
  HAL_TIM_Base_Start_IT(&htim6);

  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim16,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);


  pinit();



  a1=0b1111111111111111; //motor sayıları için pwm

  a2=400;
  a3=0;


  ms.MotorSayi=a1;
  ms.MotorDuty=a2;





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  standart();


	     //----------------------------------Dahili programları çalıştırma-------------------------------//
            //RX[0] VE RX[1]  Kontrol
	        //RX[2] RX[3] RX[4]//PRG SEÇME
	        //RX[5] //Selected
	        //RX[6] RX[7]// timer saniye
	        //RX[8] RX[9]// mduty temp duty

	  if(RxBuf[0]=='P' && RxBuf[1]=='R')
		   {
		     HAL_UART_Transmit_DMA(&huart2, "OK\n", 3);
		     RxBuf[0]=0;
		     RxBuf[1]=0;

		     timer=RxBuf[6];
		     timer=timer << 8;
		     timer=timer | 0xF0 & RxBuf[7];
		     timer=timer+2;

		     HAL_Delay(10);
             total_time=timer;
		     start=1;
		     printf("Program total time(saniye)=%d\n\r",total_time);
		     standarprgslctd(); //dutyseçimi


		   }

	    if(start==1){
	    	if(total_time>10){
	    	 program(total_time);

	    	}else start=0;

	    }else if (start==0)
	    {
	    	     a1=0b1111111111111111;  //Tüm sistem pin seçildi
	    		 a2=0;                   //duty sıfırlandı
	    		 ms.MotorSayi=a1;
	    		 ms.MotorDuty=a2;
	    		 ms.TempDuty=a2;
	    		 Pwm_Start(ms);
	    		 HAL_UART_Transmit_DMA(&huart2, "PRG OK\n\r", 8);
	    		 start=2;
	    }else{

	    }

      HAL_Delay(40);
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 2;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 2;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 374;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 63999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 2;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 2;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 1000;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
