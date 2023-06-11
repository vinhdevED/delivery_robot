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
#include "MPU6050.h"
#include "i2c-lcd.h"
#include "string.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#define TRIG_PIN GPIO_PIN_12
#define TRIG_PORT GPIOB
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
void DISTANCE_LCD(uint32_t);
void processingADC();
uint8_t HCSR04_Read(void);
int changeSpeed(int,int);
void controlSpeed(void);
void driving(void);
void rotate(void);
void DISTANCE(void);
void CAR_STOP(void);
void MPU6050(void);
#define FORWARD_0 0
#define FORWARD_1 1
#define FORWARD_2 2
int routes[3]={FORWARD_0, FORWARD_1,FORWARD_2};
int currentRoute = FORWARD_0;
//////////////////////
//hcsr04
uint32_t IC_Val1 =0;
uint32_t IC_Val2=0;
uint32_t Difference=0;
uint8_t Is_First_Captured =0;
uint8_t Distance =0;
float Dist =0.0;
uint8_t r_data;
char notiString[15]={0};
char data[15]={0};
bool isFinish =false;
bool finishRoute = false;
int routePlanning =1;
//For MPU6050 initialize
float Ax, Ay, Az; //linear acceleration
float Gx, Gy, Gz; //angular velocity
double accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
double AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
int c=0;
int i=0;

const int maxSpeed = 85;  //78
const int minSpeed = 65;   //65
double angle; //due to how I orientated my MPU6050 on my car, angle = roll
double targetAngle = 0;
int equilibriumSpeed = 75;  //71
int leftSpeedVal;
int rightSpeedVal;
bool isDriving = false; //it the car driving forward OR rotate/stationary
bool prevIsDriving = true; //equals isDriving in the previous iteration of void loop()
//bool paused = false; //is the program paused
float elapsedTime, currentTime, previousTime;
int square =0;
uint32_t readvalue;
// UART transmit buffer
char uartTxBuffer[2] ={};
static int countStraight;
int targetGyroX;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim->Instance == TIM1) {
		uint32_t detect ;
		 processingADC();
         if(readvalue<=3100){
        	 detect =0;
         }else{
        	 detect =1;
         }
		snprintf(uartTxBuffer, 2,"%lu", detect );
	    HAL_UART_Transmit(&huart1, (uint8_t*)uartTxBuffer, strlen(uartTxBuffer), 500);
	    DISTANCE_LCD(detect);
	 }
	else if(htim->Instance == TIM2){
		 Dist = HCSR04_Read();
	 }

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
		}
	}
}

void delay(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim2,0);
	while(__HAL_TIM_GET_COUNTER(&htim2)<time);
}

uint8_t HCSR04_Read (void){
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);
	return Distance;
}

void DISTANCE_LCD(uint32_t status){
	lcd_goto_XY(2, 0);
	// Clear the previous message
	lcd_send_string("                ");
	switch (status) {
		case 0:
			sprintf(notiString,"NO_PACKAGE");
			break;
		case 1:
			sprintf(notiString,"DELIVERY");
			break;
		default:
			break;
	}
	lcd_goto_XY(2, 0);
	lcd_send_string(notiString);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance ==USART1){
		//START ROBOT FOR DESTINATION A
		if(r_data == 'a'){
			isDriving =true;
		//START ROBOT FOR DESTINATION B
		}else if(r_data == 'b'){
			isDriving =true;
		//START ROBOT FOR DESTINATION C
		}else if(r_data == 'c'){
			isDriving =true;
			//angleTurnLeft();

		}
		HAL_UART_Receive_IT(&huart1, &r_data, 1);
	}

}

void angleTurnLeft(){
	targetAngle += 90;
	if (targetAngle > 180){
	targetAngle -= 360;
	}
    isFinish =true;
	isDriving = false;
}

void angleTurnRight(){
	targetAngle -= 90;
	if (targetAngle < -180){
	targetAngle += 360;
	}
	isFinish =true;
	isDriving = false;
}

void CAR_FORWARD(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);   //Start for first motor clock wise rotation
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);   //Start for second motor clock wise rotation
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}



void CAR_TURN_RIGHT(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   //Start for FIRST motor clock wise rotation
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);   //Start for SECOND motor clock wise rotation
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}
void CAR_TURN_LEFT(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);   //Start for FIRST motor clock wise rotation
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);   //Start for SECOND motor clock wise rotation
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}
void CAR_STOP(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);
}


void driving(){
	i++;
	double deltaAngle = angle - targetAngle ; //rounding is neccessary, since you never get exact values in reality
	CAR_FORWARD();
    if(deltaAngle !=0){
    	controlSpeed();
    	rightSpeedVal = equilibriumSpeed;
    	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,leftSpeedVal);
    	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,rightSpeedVal);
    }
}

void controlSpeed(){
  double deltaAngle =  angle - targetAngle;
  if(deltaAngle>=2){
	  leftSpeedVal = changeSpeed(leftSpeedVal, round(deltaAngle));
  }else if (deltaAngle<=-2){
	  leftSpeedVal = changeSpeed(leftSpeedVal, round(deltaAngle));
  }
  else{
	  leftSpeedVal = equilibriumSpeed;
	  rightSpeedVal = equilibriumSpeed;
  }

}




void rotate() {
    double deltaAngle = angle - targetAngle;
    int targetGyroX;
    if (abs(deltaAngle) <=1.5) {
        CAR_STOP();
        if (isFinish) {
           // HAL_Delay(500);
            isFinish =false;
            if(routePlanning==1){
                   currentRoute++;
            }else if(routePlanning ==2){
                   currentRoute--;
            }
                   isDriving = true;
         }
        if(finishRoute){
        	finishRoute =false;
        	isDriving=true;
        }

    } else {
    	 if (angle > targetAngle) {
    	            CAR_TURN_RIGHT();
    	} else if (angle < targetAngle) {
    	            CAR_TURN_LEFT();
    	}
        // Adjust target gyroscopic measurement
        if (abs(deltaAngle) < 45) { //10
        	targetGyroX = 2* abs(deltaAngle);

        } else {
        	targetGyroX =20;
        }
        // Proportional control adjustment
        double error = targetGyroX - abs(Gx);
        double adjustment = 0.5 * error;
        if (angle > targetAngle) {
              leftSpeedVal = changeSpeed(leftSpeedVal, -1 + adjustment);

        } else {
              leftSpeedVal = changeSpeed(leftSpeedVal, +1 + adjustment);
                     }
        rightSpeedVal = leftSpeedVal;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, leftSpeedVal);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, rightSpeedVal);
    }

}



int changeSpeed(int motorSpeed, int increment){
	motorSpeed +=increment;
	if(motorSpeed > maxSpeed){
			motorSpeed = maxSpeed;
	}else if(motorSpeed <minSpeed){
		motorSpeed = minSpeed ;
	}
	return motorSpeed;
}

void calculateError(){
	float Q = 0.01;  // Process noise covariance
	float R = 0.1;   // Measurement noise covariance

	// Initialize Kalman filter for X-axis accelerometer
	float xAccel = 0.0;  // Estimated state
	float PAccel = 0.0;  // Estimated error covariance

	// Initialize Kalman filter for Y-axis accelerometer
	float yAccel = 0.0;  // Estimated state
	float YAccel = 0.0;  // Estimated error covariance
	// Initialize Kalman filter for Z-axis accelerometer
    float zAccel = 0.0;  // Estimated state
    float ZAccel = 0.0;  // Estimated error covariance
	c = 0;
	  while (c < 200) {
	    MPU6050_Read_Accel(&Ax, &Ay, &Az);
	    HAL_Delay(1);
	    // Apply Kalman filter to X-axis accelerometer reading
	            float xAccel_pred = xAccel;
	           // float PAccel_pred = PAccel + Q;
	            float KAccel =PAccel + Q / (PAccel + Q + R);
	            xAccel = xAccel_pred + KAccel * (Ax - xAccel_pred);
	            PAccel = (1 - KAccel) * PAccel + Q;
	    // Apply Kalman filter to Y-axis accelerometer reading
	            float yAccel_pred = yAccel;
	           // float YAccel_pred = YAccel + Q;
                float KAccelY = YAccel + Q / (YAccel + Q + R);
	            yAccel = yAccel_pred + KAccelY * (Ay - yAccel_pred);
	            YAccel = (1 - KAccelY) * YAccel + Q;
	   // Apply Kalman filter to Z-axis accelerometer reading
	            float zAccel_pred = zAccel;
	            //float ZAccel_pred = ZAccel + Q;
	            float KAccelZ =ZAccel + Q / (ZAccel + Q + R);
	            zAccel = yAccel_pred + KAccelY * (Az - zAccel_pred);
	            ZAccel = (1 - KAccelZ) * ZAccel + Q;
	    // Sum all readings
	    AccErrorX += (atan((yAccel)/sqrt(pow((xAccel),2)+pow((zAccel),2)))*180/M_PI);
	    AccErrorY += (atan(-1*(xAccel)/sqrt(pow((yAccel),2)+pow((zAccel), 2)))*180/M_PI);
	    c++;
	  }
	  //Divide the sum by 200 to get the error value, since expected value of reading is zero
	  AccErrorX = AccErrorX/200.0;
	  AccErrorY = AccErrorY/200.0;
	  c = 0;

	  // Read gyro values 200 times
	    while (c < 200) {
	      MPU6050_Read_Gyro(&Gx, &Gy, &Gz);
	      HAL_Delay(1);
	      // Sum all readings
	      GyroErrorX += Gx;
	      GyroErrorY += Gy;
	      GyroErrorZ += Gz;
	      c++;
	    }
	    //Divide the sum by 200 to get the error value
	    GyroErrorX = GyroErrorX / 200.0;
	    GyroErrorY = GyroErrorY / 200.0;
	    GyroErrorZ = GyroErrorZ / 200.0;


}


void goForwardandLeft(int numberSquare, int time){
	//NUMBERSQUARE is the number of bricks robot wants to go
	//i is the time for robot go on 2 bricks and go on
	if(square <=numberSquare){
	if(Dist >15){
		if(i<time){
			if (abs(angle-targetAngle) <2){  //3
					          if (countStraight < 20){
					            countStraight ++;
					          } else {
					            countStraight = 0;
					            equilibriumSpeed = leftSpeedVal;
					          }
		    }else {
				countStraight = 0;
			}
			driving();
		}else{
			square+=4;
			i=0;
		}
	}else{
		CAR_STOP();
	}
	}else{
		square = 0;
		if((routePlanning ==1) && (currentRoute <2)){
			angleTurnLeft();
		}
		//CHECK WHEN COMPLETE THE FIRST ROUTE , ROBOT HAS BEEN FINISHED THE FIRST ROUTE THAT LEADS SECOND ROUTE
	    if((currentRoute ==2) && (routePlanning ==1)){
			isDriving = false;
			finishRoute =true;
	    }
		if((routePlanning ==2)){
			angleTurnRight();
		}


	}
}



void processing(){
	static int count;
				  if (count <6){
				    count ++;
				  } else {
				    count = 0;
				      if (isDriving != prevIsDriving){
				          leftSpeedVal = equilibriumSpeed;
				          countStraight = 0;
				      }
                      //START FOR PATH PLANNING A-B-C
					  if (isDriving) {
						  switch (currentRoute) {
							case FORWARD_0:
									goForwardandLeft(16,35); //20

								break;
							case FORWARD_1:
								goForwardandLeft(8, 20); //10
				                break;
							case FORWARD_2:
								if(r_data =='a'){
									goForwardandLeft(16, 33);
								}else if(r_data =='b'){
									goForwardandLeft(12, 20);
								}else if(r_data =='c'){
									goForwardandLeft(4, 12); //6
								}
			                    break;
						}
					  }else {
						//WHEN FINISHING FIRST ROUTE, SET ANGLE TO MAIN NAVIGATION AND START FOR SECOND ROUTE
					  	if(finishRoute){
					  		targetAngle = 0;
					  		routePlanning = 2 ;
					  	}
					  	if((routePlanning ==2)&& (currentRoute ==0)){
					  			targetAngle =0;
					  			}

					  	//WHEN DO NOTHING
					  	rotate();
					  }
					prevIsDriving = isDriving;

				  }

}


void setupMPU6050(){

	    // === Read accelerator (on the MPU6050) data === //
		MPU6050_Read_Accel(&Ax, &Ay, &Az);
		HAL_Delay(1);
		// Calculating Roll and Pitch from the accelerometer data
		accAngleX = (atan((Ay)/sqrt(pow((Ax), 2) + pow((Az), 2)))*180/ M_PI)- AccErrorX; //AccErrorX is calculated in the calculateError() function;
		accAngleY = (atan(-1*(Ax) /sqrt(pow((Ay), 2) + pow((Az), 2)))*180/ M_PI)- AccErrorY;
		// === Read gyroscope (on the MPU6050) data === //
		previousTime = currentTime;
		currentTime = HAL_GetTick()*1000;
		elapsedTime = (currentTime - previousTime)/1000000.0;

		MPU6050_Read_Gyro(&Gx,&Gy,&Gz);
        HAL_Delay(1);
		// Correct the outputs with the calculated error values
		Gx -= GyroErrorX; //GyroErrorX is calculated in the calculateError() function
		Gy -= GyroErrorY;
		Gz -= GyroErrorZ;
		// Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
		gyroAngleX += Gx * elapsedTime; // deg/s * s = deg
		gyroAngleY += Gy * elapsedTime;
		yaw += Gz * elapsedTime;

		roll = 0.98 * gyroAngleX + 0.02	* accAngleX;
		pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
		angle = round(roll * 100) / 100;
}


void processingADC(){
	HAL_ADC_Start_DMA(&hadc1, &readvalue, 1);
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  lcd_send_string("DELIVERY ROBOT");
  MPU6050_init();
  calculateError();
  HAL_Delay(20);
  HAL_UART_Receive_IT(&huart1,&r_data,1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim1);                        //Interrupt for Data light sensor
  HAL_TIM_Base_Start(&htim3);							//Initialize stm32 timer 3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);				//PB0 Start pwm second motor( 3-4) 100% duty cycle
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);				//PB1 Start pwm first motor( 1-2)  100% duty cycle
  currentTime = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
	setupMPU6050();
	processing();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 249;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 63999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
