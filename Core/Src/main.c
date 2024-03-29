/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include"stm32f4xx.h"
#define ARM_MATH_CM4

//Register Definitions for LIS302DL
#define LIS302DL_ADDR     (0x3B)
#define WHO_AM_I          (0x0F)
#define CTRL_REG1         (0x20)
#define CTRL_REG2         (0x21)
#define CTRL_REG3         (0x22)
#define HP_FILTER_RESET   (0x23)
#define STATUS_REG        (0x27)
#define OUT_X             (0x29)
#define OUT_Y             (0x2B)
#define OUT_Z             (0x2D)

// Calibration constants
#define X_OFFSET 18
#define THRESH_LOW -120
#define THRESH_HIGH 120

//User-defined Function Declarations
//to initialize and configure the GPIO pins and SPI1 respectively
void GPIO_Init(void);
void SPI_Init(void);
//to transmit and receive data on the SPI.
uint16_t SPI_Transmit(uint8_t data);
uint16_t SPI_Receive(uint8_t addr);
//initialize LIS302DL, write data to it, and read data from it respectively.
void LIS_Init(void);
void LIS_Write(uint8_t addr, uint8_t data);
void LIS_Read(void);
//to process the raw data, and create delay in milliseconds using TIM4.
int16_t Convert_To_Val(uint16_t val);
void TIM4_ms_Delay(uint16_t delay);

//User-defined variables
uint16_t x,y,z;
int16_t x_final, y_final, z_final;
uint16_t rxd,rxdf;

//Initializing and Configuring the GPIO Pins with GPIO_Init(),
void GPIO_Init(){
	// Enable GPIOA clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

//Firstly, we enable GPIOA port, by writing a 1 to position 0 in the RCC->AHB1ENR register.
//Then, PA5, PA6, and PA7 need to be put into Alternate Function (AF) mode.
//This is done by writing 10 to positions 10, 12, and 14 in the GPIOA->MODER register.

	// CS is connected to PE3, SPI1_SCK is connected to PA5,
	//SPI1_MISO is connected to PA6, and SPI1_MOSI is connected to PA7.
	// Configuring PA5, PA6, PA7 in alternate function mode
	GPIOA->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);

	// Select AF5 for SPI on PA5, PA6, PA7 (AF5=> SPI1/2)
	GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL5_2 | GPIO_AFRL_AFSEL5_0
					 | GPIO_AFRL_AFSEL6_2 | GPIO_AFRL_AFSEL6_0
					 | GPIO_AFRL_AFSEL7_2 | GPIO_AFRL_AFSEL7_0);

	// Enable GPIOE clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

	// Since PE3 is CS, it needs to be configured in Output Mode
	GPIOE->MODER |= GPIO_MODER_MODER3_0;

	GPIOA->OSPEEDR |= ( GPIO_OSPEEDER_OSPEEDR5_0 | GPIO_OSPEEDER_OSPEEDR6_0 |
			            GPIO_OSPEEDER_OSPEEDR7_0);

	GPIOA->PUPDR |= (GPIO_PUPDR_PUPD5_1 | GPIO_PUPDR_PUPD6_1 | GPIO_PUPDR_PUPD7_1);

	// Enable clock for GPIOD and Configure all leds in output mode
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	GPIOD->MODER |= (GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0);
}

void SPI_Init(){
	// Enable SPI clock
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	// Select the Master Configuration(stm32 here)

	//the STM32F407 needs to be configured as a master for that,
	//we write a 1 to position to 2 in the SPI1->CR1 register.
	//Then, we disable the bidirectional-data mode and receive only . full duplex
	//We do this by writing a 0 at positions 15 and 10 in the SPI1->CR1 register.
	SPI1->CR1 |= SPI_CR1_MSTR;

	SPI1->CR1 &= ~SPI_CR1_BIDIMODE;

	SPI1->CR1 &= ~SPI_CR1_RXONLY;

	// Set the Data Frame Format (DFF) to '0' or 8-bit.
	SPI1->CR1 &= ~SPI_CR1_DFF;

	// SSI and SSM bits in the SP1->CR1 register need to be set
	// to '1'
	SPI1->CR1 |= (SPI_CR1_SSI | SPI_CR1_SSM);

	// Setting Baud Rate
	SPI1->CR1 &= ~SPI_CR1_BR;

	// Set the transmission to MSB First Mode
	SPI1->CR1 &= ~SPI_CR1_LSBFIRST;

	// Configure CPOL and CPHASE to '0' and '0', respectively.
	// i.e. Clock is at '0' when idle, and data capture is done
	// on the first clock transition which is the rising edge.
	SPI1->CR1 &= ~SPI_CR1_CPHA;
	SPI1->CR1 &= ~SPI_CR1_CPOL;

	// Enable CRC
	SPI1->CR1 |= SPI_CR1_CRCEN;

	// Enable SPI
	SPI1->CR1 |= SPI_CR1_SPE;

	// Selecting Motorola Format
	SPI1->CR2 = 0x0000;
}

uint16_t SPI_Transmit(uint8_t data){
	//  Wait until the TX buffer is empty, i.e. data is transmitted
	while(!((SPI1->SR) & SPI_SR_TXE)){}
	// Load the data into the data register
	SPI1->DR = data;

	while(!(SPI1->SR & SPI_SR_RXNE)){}
	// If reception is intended, read the value from the data register
	rxd = SPI1->DR;

	return rxd;
}

uint16_t SPI_Receive(uint8_t addr){
	GPIOE->BSRR |= GPIO_BSRR_BR3; // BSRR(Bit Set/Reset Register)
	addr |= 0x80;
	SPI_Transmit(addr);
	rxdf = SPI_Transmit(0); //sending and storing the received data in the variable
	GPIOE->BSRR |= GPIO_BSRR_BS3;
	return rxdf;
}

void LIS_Write(uint8_t addr,uint8_t data){
	// Selecting the LIS accelerometer
	GPIOE->BSRR |= GPIO_BSRR_BR3;

	// Send the Register Address
	SPI_Transmit(addr);

	// Send the data to be written
	SPI_Transmit(data);

	// De-select the accelerometer
	GPIOE->BSRR |= GPIO_BSRR_BS3;
}

void LIS_Init(){
	// Powering on the accelerometer and Enabling the x,y and z axis for acceleration capture
	LIS_Write(CTRL_REG1, 0x47);
}

void LIS_Read(){
	// Reading the data for x-axis
	x = SPI_Receive(OUT_X);

	// Reading the data for y-axis
	y = SPI_Receive(OUT_Y);

	// Reading the data for z-axis
	z = SPI_Receive(OUT_Z);
}

void TIM4_ms_Delay(uint16_t delay){
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; //Enable the clock for TIM3
	TIM4->PSC = 16000-1; //Set the clock frequency to 1KHz
	TIM4->ARR = delay; // Get the required delay from user
	TIM4->CNT = 0;
	TIM4->CR1 |= 1; // Start the timer
	while(!(TIM4->SR & 1)){} // Wait for the "Update Interrupt Flag"
	TIM4->SR &= ~(0x0001); // Clear the "Update Interrupt Flag"
}

int16_t Convert_To_Val(uint16_t val){
	//we first check if the value obtained is negative,
	if ((val & 0x80) == 0x80){ //we perform the AND operation of val and 0x80,


		val = ~val;// if the output is 0x80 then, we obtain the magnitude of the 2's complement number.
		val += 1;
		val &= 0x00FF; //we make sure only the last 8-bits
		val = ( val * 2300 ) / 127; //multiply the magnitude with 2300, default acceleration data is-+2.3g
		                            //the value is divided by 127 since the data is 8-bits
		return (-1 * val);
	}
	else
		return (( val * 2300 ) / 127);
}

int main(void){
	GPIO_Init();
	SPI_Init();
	LIS_Init();
	GPIOE->BSRR |= GPIO_BSRR_BS3;
	while(1){
		// Call the read function
		LIS_Read();

		// Use the Convert_To_Val function to convert raw data into actual data
		x_final = Convert_To_Val(x) + X_OFFSET;
		y_final = Convert_To_Val(y);

		// Switch on LEDs based on the acceleration value obtained
		if ((x_final != 0) && (y_final != 0)){
			if (x_final > THRESH_HIGH){
				GPIOD->ODR |= GPIO_ODR_OD14;
				GPIOD->ODR &= ~(GPIO_ODR_OD12 | GPIO_ODR_OD13 | GPIO_ODR_OD15);
			}
			else if (x_final < THRESH_LOW){
				GPIOD->ODR |= GPIO_ODR_OD12;
				GPIOD->ODR &= ~(GPIO_ODR_OD14 | GPIO_ODR_OD13 | GPIO_ODR_OD15);
			}
			if (y_final > THRESH_HIGH){
				GPIOD->ODR |= GPIO_ODR_OD13;
				GPIOD->ODR &= ~(GPIO_ODR_OD12 | GPIO_ODR_OD14 | GPIO_ODR_OD15);
			}
			else if (y_final < THRESH_LOW ){
				GPIOD->ODR |= GPIO_ODR_OD15;
				GPIOD->ODR &= ~(GPIO_ODR_OD12 | GPIO_ODR_OD13 | GPIO_ODR_OD14);
			}
		}
		else
			GPIOD->ODR &= ~(GPIO_ODR_OD12 | GPIO_ODR_OD13 | GPIO_ODR_OD14 | GPIO_ODR_OD15);

		// Give a finite delay
		TIM4_ms_Delay(20);
	}
}
