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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SPEED 625 //160
#define HI_SPEED 742 //190 >742
#define TURN_SPEED 664 //170

#define forwardtime 600  //Time the robot spends turning (milliseconds)
#define turntime 900     //Time the robot spends turning (milliseconds)
#define backtime 800     //Time the robot spends turning (milliseconds)

//define steer directing

#define FRONT 90+2 //+2 adjust      // adjust this value if your steer is not facing front at beginning
#define LOW     GPIO_PIN_RESET
#define HIGH    GPIO_PIN_SET
#define OUTPUT	GPIO_MODE_OUTPUT_PP
#define INPUT	GPIO_MODE_INPUT
#define I2C_ADDR_W 0b11010110
#define I2C_ADDR_R 0b11010111
#define SHOCK_THRESHOLD 22000
#define LSM6DSL_FIFO_CTRL5      0x0A
#define LSM6DSL_FIFO_CTRL3      0x08
#define LSM6DSL_FIFO_CTRL1      0x06
#define LSM6DSL_FIFO_STATUS1    0x3A
#define LSM6DSL_FIFO_DATA_OUT_L 0x3E
#define LSM6DSL_FIFO_DATA_OUT_H 0x3F
#define LSM6DSL_FIFO_THRESHOLD  32
#define LSM6DSL_ODR_FIFO_104HZ  0x05
int LEFT = FRONT - 30;
int SLIGHT_LEFT = FRONT - 20;
int RIGHT = FRONT + 30;
int SLIGHT_RIGHT = FRONT + 20;
int angle = FRONT;
int turn_flag = 0;
#define SENSOR_FRONT 90+10 //+10 adjust
int SENSOR_LEFT = SENSOR_FRONT + 25;
int SENSOR_RIGHT = SENSOR_FRONT - 25;
int SENSOR_FAR_LEFT = SENSOR_FRONT + 60;
int SENSOR_FAR_RIGHT = SENSOR_FRONT - 60;
int distance;
int numcycles = 0;
int leftscanval, centerscanval, rightscanval, ldiagonalscanval,
		rdiagonalscanval;
int tim_flg = 0;
const int DISTANCE_LIMIT = 30;  //distance limit for obstacles in front
const int SIDE_DISTANCE_LIMIT = 30;
#define LPT 2  // scan loop counter
int acc[128];
char rxbuf[200];
char msgbuf[210];
uint8_t i2cbuf[128];
char c = 0;
uint8_t compare_sign;
uint8_t compare_sign_count = 0;
uint8_t final_turn = 0;
int old_front_distance = 0;
int front_distance_count = 0;
uint8_t shock_A = 0;
uint8_t shock_B = 0;
int new_count, count_diff, old_count, point_count;
int old_point_time, new_point_time, point_diff;
typedef struct {
	GPIO_TypeDef *GPIO;
	uint16_t PIN;
} GPIO_T;
typedef struct {
	int x;
	int y;
	int z;
} accel_data;
GPIO_T head_pin = { GPIOB, GPIO_PIN_11 }; //PB11
GPIO_T head_sensor_pin = { GPIOA, GPIO_PIN_7 }; //PA7
GPIO_T ENA_pin = { GPIOB, GPIO_PIN_8 }; //PB8
GPIO_T IN1 = { GPIOC, GPIO_PIN_7 };
GPIO_T IN2 = { GPIOA, GPIO_PIN_4 };
GPIO_T Trg = { GPIOC, GPIO_PIN_13 };
GPIO_T Echo = { GPIOB, GPIO_PIN_2 };
GPIO_T LFSensor_0 = { GPIOC, GPIO_PIN_0 };
GPIO_T LFSensor_1 = { GPIOC, GPIO_PIN_1 };
GPIO_T LFSensor_2 = { GPIOC, GPIO_PIN_2 };
GPIO_T LFSensor_3 = { GPIOC, GPIO_PIN_4 };
GPIO_T LFSensor_4 = { GPIOC, GPIO_PIN_5 };
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM7_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
static void SendStringUSB(char *buf);
static void SendFirstBee(char *buf);
static HAL_StatusTypeDef RecvFirstBee(char *buf, int bufsize);
static void ExecATcmd(char *atcmd);
static void SendATcmd(char *atcmd);
void pinMode(GPIO_T pin, uint32_t mode);
void digitalWrite(GPIO_T gpio, GPIO_PinState PinState);
int angleTOduty(int angle);
void analogWrite(GPIO_T *pin, int duty);
void PWMServo_write(GPIO_T *servo_pin, int angleArg);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void SendStringUSB(char *buf) {
	uint8_t err_count = 3;
	while (CDC_Transmit_FS((uint8_t*) buf, strlen(buf)) != USBD_OK) {
		HAL_Delay(10);
		err_count--;
		if (!err_count)
			break;
	}
}

static void SendFirstBee(char *buf) {
	uint8_t err_count = 3;
	while (HAL_UART_Transmit(&huart2, (uint8_t*) buf, strlen(buf), 100)
			!= HAL_OK) {
		HAL_Delay(10);
		err_count--;
		if (!err_count)
			break;
	}
}

static HAL_StatusTypeDef RecvFirstBee(char *buf, int bufsize) {
	uint8_t err_count = 4;
	memset(buf, 0, bufsize);
	while (HAL_UART_Receive(&huart2, (uint8_t*) buf, bufsize - 1, 100) != HAL_OK) {
		HAL_Delay(10);
		err_count--;
		if (!err_count) {
			break;
		}
	}
	return HAL_OK;
}

static void SendATcmd(char *atcmd) {
	char txbuf[128];

	strncpy(txbuf, atcmd, sizeof(txbuf));
	//USB message transmit
	SendStringUSB(txbuf);
	HAL_Delay(10);
	SendFirstBee(txbuf);
}

static void ExecATcmd(char *atcmd) {

	int i = 0;
	char rxbuf[300];
	char rb[128];

	rxbuf[0] = 0;

	SendATcmd(atcmd);
	while (1) {
		RecvFirstBee(rb, sizeof(rb));
		if (strlen(rb) > 0) {
//			SendStringUSB(rb);
			strcat(rxbuf, rb);
			if (strstr(rxbuf, "OK") != 0)
				break;
			strcpy(rxbuf, rb);
		}
		if (i < 3) {
			i++;
			HAL_Delay(5);
		} else {
			break;
		}
	}
}
void BufClear(char *msgbuf) {
	for (int i = 0; i < sizeof(msgbuf); i++) {
		msgbuf[i] = 0;
	}
}
void Bufstrcpy(char *buf, const char *src) {
	BufClear(buf);
	strcpy(buf, src);
}
int angleTOduty_bak(int angle) {
	double old_min = 0.0, old_max = 180.0;
	double new_min = 25.0, new_max = 120.0;

	// Convert input to double
	double angle_double = (double) angle;

	// Normalize
	double normalized = (angle_double - old_min) / (old_max - old_min);

	// Scale and shift
	double new_value_double = normalized * (new_max - new_min) + new_min;

	// Convert back to int
	int new_value = (int) (new_value_double + 0.5); // adding 0.5 for rounding

	return new_value;
}
float map(float value, float old_min, float old_max, float new_min,
		float new_max) {
	return (value - old_min) / (old_max - old_min) * (new_max - new_min)
			+ new_min;
}
int angleTOduty(int angle) {
	float old_value = (float) angle;
	float new_value = map(old_value, 0, 180, 25, 120);

	return (int) new_value;
}
void analogWrite(GPIO_T *pin, int duty) {
//timer period: 48MHz /50Hz 960000 period 1000 ,960
	if (pin == &head_pin) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty);
	} else if (pin == &head_sensor_pin) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
	} else if (pin == &ENA_pin) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, duty);
	}
}
void PWMServo_write(GPIO_T *servo_pin, int angleArg) {
	if (angleArg < 0)
		angleArg = 0;
	if (angleArg > 180)
		angleArg = 180;
	int duty = angleTOduty(angleArg);

	analogWrite(servo_pin, duty);
}
void digitalWrite(GPIO_T gpio, GPIO_PinState PinState) {
	HAL_GPIO_WritePin(gpio.GPIO, gpio.PIN, PinState);
}
void go_Back() { //back
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);
}

void go_Advance()  //Forward
{
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, HIGH);
}

void stop_Stop()  //Stop
{
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
	analogWrite(&ENA_pin, 0);
}

void steer(int angle) {
	if (angle < LEFT)
		angle = LEFT;
	if (angle > RIGHT)
		angle = RIGHT;
	PWMServo_write(&head_pin, angle);
}
void delayMicroseconds(uint32_t time) {
	uint32_t chktime;
	chktime = __HAL_TIM_GET_COUNTER(&htim7) + time;
	while (__HAL_TIM_GET_COUNTER(&htim7) < chktime)
		;
}

uint32_t watch(void) {
	uint32_t distance = 0;
	HAL_TIM_Base_Start(&htim7);
	__disable_irq();
	HAL_GPIO_WritePin(Trg.GPIO, Trg.PIN, GPIO_PIN_RESET);
	delayMicroseconds(5);
	HAL_GPIO_WritePin(Trg.GPIO, Trg.PIN, GPIO_PIN_SET);
	delayMicroseconds(15);
	HAL_GPIO_WritePin(Trg.GPIO, Trg.PIN, GPIO_PIN_RESET);
	while (HAL_GPIO_ReadPin(Echo.GPIO, Echo.PIN) == GPIO_PIN_RESET)
		;
	__HAL_TIM_SET_COUNTER(&htim7, 0);
	while (HAL_GPIO_ReadPin(Echo.GPIO, Echo.PIN) == GPIO_PIN_SET) {
		if (__HAL_TIM_GET_COUNTER(&htim7) > 23000) {
			distance = __HAL_TIM_GET_COUNTER(&htim7) * 343 / 10000 / 2;
			__enable_irq();
//			SendStringUSB("no obstacle");
			return distance;
		}
	}
	distance = __HAL_TIM_GET_COUNTER(&htim7) * 343 / 10000 / 2;
	__enable_irq();
	return distance;
}
float watch_f(void) {
	float distance = 0;
	HAL_TIM_Base_Start(&htim7);
	__disable_irq();
	HAL_GPIO_WritePin(Trg.GPIO, Trg.PIN, GPIO_PIN_RESET);
	delayMicroseconds(5);
	HAL_GPIO_WritePin(Trg.GPIO, Trg.PIN, GPIO_PIN_SET);
	delayMicroseconds(15);
	HAL_GPIO_WritePin(Trg.GPIO, Trg.PIN, GPIO_PIN_RESET);
//	while (HAL_GPIO_ReadPin(Echo.GPIO, Echo.PIN) == GPIO_PIN_RESET)
//		;
	__HAL_TIM_SET_COUNTER(&htim7, 0);
	while (HAL_GPIO_ReadPin(Echo.GPIO, Echo.PIN) == GPIO_PIN_SET) {
		if (__HAL_TIM_GET_COUNTER(&htim7) > 23000) {
			distance = __HAL_TIM_GET_COUNTER(&htim7) * 343 / 10000 / 2;
			__enable_irq();
//			SendStringUSB("no obstacle");
			return distance;
		}
	}
	distance = __HAL_TIM_GET_COUNTER(&htim7) * 343 / 10000 / 2;
	__enable_irq();
	return distance;
}
uint8_t watchsurrounding(void) {
	uint8_t obstacle_status = 0b00000;
	int centerscanval = watch();
	if (old_front_distance == centerscanval) {
		front_distance_count++;
	} else {
		old_front_distance = centerscanval;
		front_distance_count = 0;
	}
	if (centerscanval < DISTANCE_LIMIT) {
		stop_Stop();
		obstacle_status = obstacle_status | 0b00100;
	}

	PWMServo_write(&head_sensor_pin, SENSOR_LEFT);
	HAL_Delay(100);
	int ldiagonalscanval = watch();
	if (ldiagonalscanval < DISTANCE_LIMIT) {
		stop_Stop();
		obstacle_status = obstacle_status | 0b01000;
	}

	PWMServo_write(&head_sensor_pin, SENSOR_FAR_LEFT);
	HAL_Delay(300);
	int leftscanval = watch();
	if (leftscanval < SIDE_DISTANCE_LIMIT) {
		stop_Stop();
		obstacle_status = obstacle_status | 0b10000;
	}

	PWMServo_write(&head_sensor_pin, SENSOR_FRONT);
	HAL_Delay(100);
	centerscanval = watch();
	if (centerscanval < DISTANCE_LIMIT) {
		stop_Stop();
		obstacle_status = obstacle_status | 0b00100;
	}

	PWMServo_write(&head_sensor_pin, SENSOR_RIGHT);
	HAL_Delay(100);
	int rdiagonalscanval = watch();
	if (rdiagonalscanval < DISTANCE_LIMIT) {
		stop_Stop();
		obstacle_status = obstacle_status | 0b00010;
	}

	PWMServo_write(&head_sensor_pin, SENSOR_FAR_RIGHT);
	HAL_Delay(100);
	int rightscanval = watch();
	if (rightscanval < SIDE_DISTANCE_LIMIT) {
		stop_Stop();
		obstacle_status = obstacle_status | 0b00001;
	}

	PWMServo_write(&head_sensor_pin, SENSOR_FRONT);
	HAL_Delay(300);
	return obstacle_status;
}
void final_go_Back(void) {
	steer(RIGHT);
	go_Back();
	analogWrite(&ENA_pin, SPEED);
	HAL_Delay(backtime);
	steer(FRONT);
}
void final_go_Advance(void) {
	steer(LEFT);
	go_Advance();
	analogWrite(&ENA_pin, HI_SPEED);
	HAL_Delay(backtime * 3 / 2);
	steer(FRONT);
}
void auto_avoidance(void) {
	++numcycles;
	uint8_t obstacle_sign = 0;
	if (numcycles >= LPT) { //Watch if something is around every LPT loops while moving forward
		stop_Stop();
		obstacle_sign = watchsurrounding();
		if (compare_sign == obstacle_sign) {
			compare_sign_count++;
		} else {
			compare_sign = obstacle_sign;
			compare_sign_count = 0;
		}
		if (obstacle_sign == 0b00000) {
//			SendStringUSB("FORWARD");
			go_Advance();
			steer(FRONT);
			analogWrite(&ENA_pin, SPEED);
			HAL_Delay(forwardtime);
			stop_Stop();
		} else if (obstacle_sign == 0b01000 || obstacle_sign == 0b11000
				|| obstacle_sign == 0b10000) {
//			SendStringUSB("hand right");
			go_Advance();
			steer(RIGHT);
			analogWrite(&ENA_pin, SPEED);
			HAL_Delay(turntime);
			stop_Stop();
			steer(FRONT);
		}

		else if (obstacle_sign == 0b00010 || obstacle_sign == 0b00011
				|| obstacle_sign == 0b00001) {
//			SendStringUSB("hand left");

			go_Advance();
			steer(LEFT);
			analogWrite(&ENA_pin, HI_SPEED);
			HAL_Delay(turntime);
			stop_Stop();
			steer(FRONT);
		}

		else if (obstacle_sign == 0b00111 || obstacle_sign == 0b01111
				|| obstacle_sign == 0b10111 || obstacle_sign == 0b11111
				|| obstacle_sign == 0b00110 || obstacle_sign == 0b01010
				|| obstacle_sign == 0b00101) {
//			SendStringUSB("hand back right");

			go_Back();
			steer(RIGHT);
			analogWrite(&ENA_pin, HI_SPEED);
			HAL_Delay(backtime);
			stop_Stop();
			steer(FRONT);
		}

		else if (obstacle_sign == 0b00100 || obstacle_sign == 0b10100
				|| obstacle_sign == 0b01100 || obstacle_sign == 0b11100
				|| obstacle_sign == 0b11011 || obstacle_sign == 0b11101
				|| obstacle_sign == 0b11110 || obstacle_sign == 0b01110) {
//			SendStringUSB("hand back left");
			go_Back();
			steer(LEFT);
			analogWrite(&ENA_pin, HI_SPEED);
			HAL_Delay(backtime);
			stop_Stop();
			steer(FRONT);
		}

		else
			SendStringUSB("no handle");
		numcycles = 0;  //Restart count of cycles
	} else {
		steer(FRONT);
		go_Advance();
		analogWrite(&ENA_pin, SPEED);
		HAL_Delay(backtime);
		stop_Stop();
	}
	if ((compare_sign_count == 3) && (front_distance_count >= 2)) {
		final_go_Back();
		steer(FRONT);
		stop_Stop();
		go_Advance();
	}
	if ((compare_sign_count >= 4) || (front_distance_count >= 3)) {
		steer(FRONT);
		go_Back();
		analogWrite(&ENA_pin, SPEED);
		HAL_Delay(backtime);
		final_go_Advance();
		stop_Stop();
		compare_sign_count = 0;
		front_distance_count = 0;
	}

}
uint8_t digitalRead(GPIO_T readpin) {
	return HAL_GPIO_ReadPin(readpin.GPIO, readpin.PIN);
}

void read_accelerometer(short int *data) {

	while (1) {
		i2cbuf[0] = 0;
		HAL_I2C_Mem_Read(&hi2c1, I2C_ADDR_R, 0x1e, I2C_MEMADD_SIZE_8BIT, i2cbuf,
				1, 30);
		if ((i2cbuf[0] & 0x01) == 0x01) {
			break;
		}
		HAL_Delay(10);
	}
	HAL_I2C_Mem_Read(&hi2c1, I2C_ADDR_R, 0x28, I2C_MEMADD_SIZE_8BIT, i2cbuf, 6,
			30);
	data[0] = i2cbuf[0] | (i2cbuf[1] << 8);
	data[1] = i2cbuf[2] | (i2cbuf[3] << 8);
	data[2] = i2cbuf[4] | (i2cbuf[5] << 8);
	snprintf(msgbuf, sizeof(msgbuf), "acc=(%hd, %hd, %hd)\r\n", data[0],
			data[1], data[2]);
	CDC_Transmit_FS((uint8_t*) msgbuf, strlen(msgbuf));
}
void SendAccelDataUSB(void) {
	char buf[100];
	short int data[3];
// Read accelerometer data
	read_accelerometer(data);

// Format accelerometer data as a string
	sprintf(buf, "Accel X: %d, Accel Y: %d, Accel Z: %d\r\n", data[0], data[1],
			data[2]);

// Send the string over USB
	SendStringUSB(buf);
}
void SendFIFODataUSB(accel_data data) {
	char buf[100];
	sprintf(buf, "Accel X: %d, Accel Y: %d, Accel Z: %d\r\n", data.x, data.y,
			data.z);
	SendStringUSB(buf);
}
void timeshow(void) {
	point_count = 0;
	new_count = __HAL_TIM_GET_COUNTER(&htim6);
	count_diff = new_count - old_count;
	old_count = new_count;
	old_point_time = new_count;
	sprintf(msgbuf, "\r\n%dms", count_diff);
	SendStringUSB(msgbuf);
}
void timepoint(void) {
	point_count++;
	int new_point_time = (__HAL_TIM_GET_COUNTER(&htim6));
	point_diff = new_point_time - old_point_time;
	old_point_time = new_point_time;
	sprintf(msgbuf, "%d %d,", point_count, point_diff);
	SendStringUSB(msgbuf);
}
void init_lsm6dsl(void) {
	uint8_t reg;

//	reg = 0b01000000; //BDU set 1
//	HAL_I2C_Mem_Write(&hi2c1, I2C_ADDR_W, 0x12, 1, &reg, 1, 1000);

	reg = 0b00000001; //// DEC_FIFO_GYRO[2:0] = '001' and DEC_FIFO_XL[2:0] = '001' for no decimation
	HAL_I2C_Mem_Write(&hi2c1, I2C_ADDR_W, LSM6DSL_FIFO_CTRL3, 1, &reg, 1, 1000);

// Set FIFO threshold to 32
	reg = LSM6DSL_FIFO_THRESHOLD;
	HAL_I2C_Mem_Write(&hi2c1, I2C_ADDR_W, LSM6DSL_FIFO_CTRL1, 1, &reg, 1, 1000);

// Enable FIFO and set mode to continuous 0b110 Set ODR to 104Hz 0b100
	reg = 0b00100110;
	HAL_I2C_Mem_Write(&hi2c1, I2C_ADDR_W, LSM6DSL_FIFO_CTRL5, 1, &reg, 1, 1000);
}

void read_lsm6dsl(void) {
	accel_data accel[32];
//	uint8_t status;
	uint8_t data[192];  // For 32 points * 6 bytes each
// Check if FIFO threshold has been reached
//	HAL_I2C_Mem_Read(&hi2c1, I2C_ADDR_R, LSM6DSL_FIFO_STATUS1, 1, &status, 1,
//			100);
//	if (status >= LSM6DSL_FIFO_THRESHOLD) {
// Read 192 bytes (32 points * 6 bytes each) of accelerometer data from FIFO
	HAL_I2C_Mem_Read(&hi2c1, I2C_ADDR_R, LSM6DSL_FIFO_DATA_OUT_L, 1, data,
			sizeof(data), 100);

// Parse the data
	for (int i = 0; i < 32; i++) {
		accel[i].x = (int16_t) (data[i * 6 + 1] << 8 | data[i * 6]);
		accel[i].y = (int16_t) (data[i * 6 + 3] << 8 | data[i * 6 + 2]);
		accel[i].z = (int16_t) (data[i * 6 + 5] << 8 | data[i * 6 + 4]);
		SendFIFODataUSB(accel[i]);
	}
	for (int i = 0; i < 32; i++) {
		// Calculate the magnitude of the acceleration vector
		int magnitude = sqrt(
				accel[i].x * accel[i].x + accel[i].y * accel[i].y
						+ accel[i].z * accel[i].z);
		if (magnitude > SHOCK_THRESHOLD) {
			if (accel[i].x > 0) {
				shock_A = 1;
			} else {
				shock_B = 1;
			}
			break;
		}

	}
}
uint32_t micros(void) {
	return __HAL_TIM_GET_COUNTER(&htim7);
}
void PID_mode(void) {
	const float Target = 15.0;
	const float Kp = 10.0;
	const float Ki = 0.15;
	const float Kd = 0.1;
	float P, I, D, preP, pretime = 0.0;
	int duty;
	while (1) {
		// get the current sensor reading
		float currentDistance = watch_f();
		// calculate the time difference
		float time_difference = (micros() - pretime) / 1000000;
		while (time_difference < 0) {
			pretime = micros();
			time_difference = (micros() - pretime) / 1000000;
		}
		// calculate the Proportional term
		P = currentDistance - Target;
		// calculate the Integral term
		I += P * time_difference;
		// calculate the Derivative term
		D = (P - preP) / time_difference;
		preP = P;
		// calculate the total error
		duty = abs(Kp * P + Ki * I + Kd * D);
		duty = map(duty, 0, 255, 0, 1000);
		// move the robot based on the total error
		if (P > 0) {
			go_Advance();
			analogWrite(&ENA_pin, duty);
		} else if (P < 0) {
			go_Back();
			analogWrite(&ENA_pin, duty);
		} else {
			stop_Stop();
		}
		pretime = micros();
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USB_DEVICE_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
	MX_TIM3_Init();
	MX_TIM7_Init();
	MX_I2C1_Init();
	MX_TIM6_Init();
	/* USER CODE BEGIN 2 */
//First Bee start
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // PWM timer start
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_Base_Start(&htim6);
	HAL_TIM_Base_Start(&htim7);
	PWMServo_write(&head_sensor_pin, SENSOR_FRONT);
	steer(FRONT);
	HAL_Delay(2000);


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		PID_mode();
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}  //end of outer while

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 12;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 960 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
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
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 960 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
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
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 960 - 1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 1000;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
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
static void MX_TIM6_Init(void) {

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 48000 - 1;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 65535;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void) {

	/* USER CODE BEGIN TIM7_Init 0 */

	/* USER CODE END TIM7_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM7_Init 1 */

	/* USER CODE END TIM7_Init 1 */
	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 48 - 1;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 65535;
	htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM7_Init 2 */

	/* USER CODE END TIM7_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC13 PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PC0 PC1 PC2 PC4
	 PC5 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4
			| GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
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
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
