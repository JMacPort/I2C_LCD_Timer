#include "main.h"

int main() {


	while (1) {

	}
}

void GPIO_Init() {
	RCC -> APB1ENR |= (1 << 1);					// GPIOB Clock initialize
}
