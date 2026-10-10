#include "sensor/dht22.h"
#include "gpio_macro.h"

#define DHT_PIN 0

typedef enum
{
	DHT_IDLE,
	DHT_START,
	DHT_WAIT_RESPONSE,
	DHT_READ_DATA,
	DHT_DONE
} DHTState_t;

static volatile DHTState_t dhtState = DHT_IDLE;
static volatile uint8_t dhtData[5];
static volatile uint8_t dhtBitIdx = 0;
static volatile uint8_t dhtReadStartCapture = 0;
static volatile uint32_t dhtRiseTime = 0;
static volatile uint32_t dhtFallTime = 0;

//Input Capture cho DHT22
void TIM5_IRQHandler(void) {
	if(!(TIM5->SR & TIM_SR_CC1IF)) return;
	uint16_t capture = TIM5->CCR1;
	uint8_t pinHigh = (GPIOA->IDR & (1<<DHT_PIN)) ? 1 : 0;

	if(dhtState != DHT_READ_DATA) return;

	if(pinHigh) {
		dhtRiseTime = capture;
	} else {
		dhtFallTime = capture;
		uint16_t width = (uint16_t)(dhtFallTime - dhtRiseTime);
		uint8_t bit = (width > 40) ? 1 : 0;
		dhtData[dhtBitIdx >> 3] <<= 1;
		dhtData[dhtBitIdx >> 3] |= bit;
		dhtBitIdx++;

		if(dhtBitIdx >= 40) {
			TIM5->DIER &= ~TIM_DIER_CC1IE;
			dhtState = DHT_DONE;
		}
	}
}

void DHT_GPIO_Init(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->OTYPER &= ~(1<<DHT_PIN);
}

static inline void DHT_PinAsOutput(void) {
	GPIOA->MODER &= ~(3 << (DHT_PIN*2));
	GPIOA->MODER |= (1 << (DHT_PIN*2));
}

static inline void DHT_PinAsTimerCapture(void) {
	GPIOA->MODER &= ~(3 << (DHT_PIN*2));
	GPIOA->MODER |= (2 << (DHT_PIN*2)) ;
	GPIOA->AFR[0] &= ~(0xF << (DHT_PIN*4));
	GPIOA->AFR[0] |= (2 << (DHT_PIN*4));
}

void DHT_TIM_Init(void) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	TIM5->PSC = 84 - 1;
	TIM5->ARR = 0xFFFF;

	TIM5->CCMR1 = (TIM5->CCMR1 & ~TIM_CCMR1_CC1S) | TIM_CCMR1_CC1S_0;
	TIM5->CCMR1 &= ~TIM_CCMR1_IC1F;
	TIM5->CCER &= ~TIM_CCER_CC1E;
	TIM5->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC1NP);
	TIM5->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM5_IRQn);
}

void DHT_CheckTimeout(void) {
	if(dhtState == DHT_READ_DATA) {
		uint16_t elapsed = (uint16_t)(TIM5->CNT - dhtReadStartCapture);
		if(elapsed > 10000) {
			TIM5->DIER &= ~TIM_DIER_CC1IE;
			TIM5->CCER &= ~TIM_CCER_CC1E;
			dhtState = DHT_IDLE;
		}
	}
}

static void DHT_Delay(uint16_t us) {
	uint16_t start = TIM5->CNT;
	while ((uint16_t)(TIM5->CNT - start) < us);
}

uint8_t DHT_StartRead(void) {
	dhtState = DHT_START;
	dhtBitIdx = 0;
	for(uint8_t i = 0; i < 5; i++) dhtData[i] = 0;

	TIM5->DIER &= ~TIM_DIER_CC1IE;
	TIM5->CCER &= ~TIM_CCER_CC1E;

	DHT_PinAsOutput();
	PINRESET(GPIOA, DHT_PIN);
	DHT_Delay(1200);
	DHT_PinAsTimerCapture();

	uint16_t t0;

	t0 = TIM5->CNT;
	while(!(GPIOA->IDR & (1<<DHT_PIN))) {
		if((uint16_t)(TIM5->CNT - t0) > 100) {
			dhtState = DHT_IDLE;
			return 0;
		}
	}

	t0 = TIM5->CNT;
	while((GPIOA->IDR & (1<<DHT_PIN))) {
		if((uint16_t)(TIM5->CNT - t0) > 100) {
			dhtState = DHT_IDLE;
			return 0;
		}
	}

	t0 = TIM5->CNT;
	while(!(GPIOA->IDR & (1<<DHT_PIN))) {
		if((uint16_t)(TIM5->CNT - t0) > 150) {
			dhtState = DHT_IDLE;
			return 0;
		}
	}

	t0 = TIM5->CNT;
	while((GPIOA->IDR & (1<<DHT_PIN))) {
		if((uint16_t)(TIM5->CNT - t0) > 150) {
			dhtState = DHT_IDLE;
			return 0;
		}
	}

	dhtState = DHT_READ_DATA;
	dhtReadStartCapture = TIM5->CNT;
	TIM5->SR = ~TIM_SR_CC1IF;
	TIM5->CCER |= TIM_CCER_CC1E;
	TIM5->DIER |= TIM_DIER_CC1IE;

	return 1;
}

uint8_t DHT_Process(float *thermal, float *humidity) {
	if(dhtState != DHT_DONE) return 0;

	uint8_t checksum = (uint8_t)(dhtData[0] + dhtData[1] + dhtData[2] + dhtData[3]);
	if (checksum != dhtData[4]) {
		dhtState = DHT_IDLE;
		return 0;
	}

	uint16_t rawHumidity = (dhtData[0]<<8) | dhtData[1];
	uint16_t rawThermal = (dhtData[2]<<8) | dhtData[3];

	*thermal = (rawThermal & 0x8000) ? -((rawThermal & 0x7FFF)/10.0f) : (rawThermal/10.0f);
	*humidity = rawHumidity / 10.0f;

	dhtState = DHT_IDLE;
	return 1;
}
