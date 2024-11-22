#include "main.h"

// LCD
#define LCD_ADDR  			0x27			// LCD Address
#define LCD_EN 	  			0x04			// Enable
#define LCD_RW    			0x02			// Read/Write
#define LCD_RS    			0x01			// Register Select
#define LCD_BL	  			0x08			// Back-light

// LCD Commands
#define LCD_CLEAR 			0x01			// Clear Screen
#define LCD_HOME  			0x02			// Cursor Home
#define LCD_ENTRY_MODE  	0x06			// Increments cursor, no shift
#define LCD_DISPLAY_ON  	0x0C			// Display on, cursor off, blink off
#define LCD_FUNCTION_SET 	0x28			// 4-bit mode, 2 lines, 5x8 font

//Prototypes
void SYS_Init();
uint16_t ADC_Read();

int main() {
	SYS_Init();

	while (1) {

	}
}

void GPIO_Init() {
	RCC -> AHB1ENR |= (1 << 0);										// GPIOA Clock Initialize

	GPIOA -> MODER &= ~(3 << (0 * 2));								// Set mode for PA0 to be analog mode; Potentiometer
	GPIOA -> MODER |= (3 << (0 * 2));

	RCC -> AHB1ENR |= (1 << 1);										// GPIOB Clock Initialize

	GPIOB -> MODER &= ~((3 << (4 * 2)) | (3 << (5 * 2))); 			// Set mode for PB4 & 5 to input mode; Buttons

	GPIOB -> PUPDR &= ~((3 << (4 * 2)) | (3 << (5 * 2)));			// Configure buttons to be Pull-Up
	GPIOB -> PUPDR |= (3 << (4 * 2)) | (3 << (5 * 2));

	GPIOB -> MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2)));			// PB8 & 9 set to Alternate Function
	GPIOB -> MODER |= (2 << (8 * 2)) | (2 << (9 * 2));

	GPIOB -> OTYPER |= (1 << 8) | (1 << 9);							// Sets PB8 & PB9 to Open-Drain

	GPIOB -> PUPDR &= ~((3 << (8 * 2)) | (3 << (9 * 2)));			// Sets Pull-Up
	GPIOB -> PUPDR |= (1 << (8 * 2)) | (1 << (9 * 2));

	GPIOB -> AFR[1] &= ~((0xF << 0) | (0xF << 1));					// Sets AF4 for I2C1 Communication
	GPIOB -> AFR[1] |= (4 << 0) | (4 << 1);
}

void ADC_Init() {
	RCC -> APB2ENR |= (1 << 8);										// ADC1 Clock Initialize

	ADC1 -> CR1 &= ~(0x3 << 24);									// 12-bit Resolution

	ADC1 -> SMPR2 &= ~(7 << 0);										// Sets ADC1 Channel 0 to 84 cycles
	ADC1 -> SMPR2 |= (4 << 0);

	ADC1 -> SQR1 &= ~(0xF << 20);									// Sets only 1 conversion

	ADC1 -> CR2 |= (1 << 0);										// Enable ADC Converter
}

void Interrupt_Init() {
	RCC -> APB2ENR |= (1 << 14);									// Enable SYSCFG

	SYSCFG -> EXTICR[2] |= (1 << 0) | (1 << 1);						// Interrupts on PB4 & 5

	EXTI -> IMR |= (1 << 4) | (1 << 5);								// Interrupt Mask
	EXTI -> RTSR |= (1 << 4) | (1 << 5);							// Rising Trigger

	NVIC -> ISER[0] |= (1 << 10) | (1 << 23);						// NVIC Interrupts on

}

void I2C_Init() {
	RCC -> APB1ENR |= (1 << 21);									// I2C1 Clock Enable

	I2C1 -> CR1 |= (1 << 15);										// Toggles SWRST; Resets the I2C connection
	I2C1 -> CR1 &= ~(1 << 15);										// Disables SWRST; I2C is ready

	I2C1 -> CR2 |= (42 << 0);										// Frequency; SYS Clock / 4 = 42 MHz
	I2C1 -> CCR = 210;												// Sets communication speed; 42 MHz / (2 * 100kHz (desired speed))

	I2C1 -> TRISE = 43;												// Maximum rise time; 1 + frequency

	I2C1 -> CR1 |= (1 << 0);										// Enable I2C1
}

void SYS_Init() {
	GPIO_Init();
	ADC_Init();
	Interrupt_Init();
	I2C_Init();
}

uint16_t ADC_Read() {
	ADC1 -> CR2 |= (1 << 30);										// Starts conversion
	while(!(ADC1 -> SR & (1 << 1)));								// Waits for conversion
	uint16_t adc_value = ADC1 -> DR;								// Stores ADC value
	ADC1 -> SR &= ~(1 << 1);										// Clears flag
	return adc_value;
}

uint8_t I2C_Check_Busy() {
	if (I2C1 -> SR1 & (1 << 1)) {									// Returns 1 if bus is busy, 0 if not
		return 1;
	}
	return 0;
}

void I2C_Start() {
	I2C1 -> CR1 |= (1 << 8);										// Generates Start

	while (!(I2C1 -> SR1 & (1 << 0)));
}

void I2C_Send_Address(uint8_t address, uint8_t read) {				// Sends device address
	I2C1 -> DR = (address << 1) | read;

	while (!(I2C1 -> SR1 & (1 << 1)));

	uint8_t temp = I2C1 -> SR1;										// Reads registers
	temp = I2C1 -> SR2;
	(void) temp;
}

void I2C_Send_Data(uint8_t data) {
	while (!(I2C1 -> SR1 & (1 << 7)));

	I2C1 -> DR = data;												// Sends data

	while (!(I2C1 -> SR1 & (1 << 2)));
}

void I2C_Stop() {
	I2C1 -> CR1 |= (1 << 9);										// Generates stop
}

void I2C_Write(uint8_t address, uint8_t data) {
	I2C_Start();													// Writes byte to device
	I2C_Send_Address(address, 0);
	I2C_Send_Data(data);
	I2C_Stop();
}

void LCD_Send_Command(uint8_t cmd) {
	uint8_t upper = (cmd & 0xF0) | LCD_BL;							// 4-bit Sequences to send commands
	uint8_t lower = ((cmd << 4) & 0xF0) | LCD_BL;

	I2C_Write(LCD_ADDR, upper | LCD_EN);
	I2C_Write(LCD_ADDR, upper);

	I2C_Write(LCD_ADDR, lower | LCD_EN);
	I2C_Write(LCD_ADDR, lower);
}

void LCD_Init() {
	for (volatile int i = 0; i < 5000; i++);						// Initializes the LCD with 4-bit mode and clears screen

	LCD_SendCommand(0x33);
	LCD_SendCommand(0x32);
	LCD_SendCommand(LCD_FUNCTION_SET);
	LCD_SendCommand(LCD_DISPLAY_ON);
	LCD_SendCommand(LCD_ENTRY_MODE);
	LCD_SendCommand(LCD_CLEAR);
}

void LCD_Send_Data(uint8_t cmd) {
	uint8_t upper = (cmd & 0xF0) | LCD_BL;							// 4-bit Sequences to send data
	uint8_t lower = ((cmd << 4) & 0xF0) | LCD_BL;

	I2C_Write(LCD_ADDR, upper | LCD_RS);
	I2C_Write(LCD_ADDR, upper);

	I2C_Write(LCD_ADDR, lower | LCD_RS);
	I2C_Write(LCD_ADDR, lower);
}





















