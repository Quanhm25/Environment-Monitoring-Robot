#include "motor.h"

#define PWM_ARR 4199
#define MOTOR_STOP 1
#define DEFAULT_SPEED 70

// Declaration

typedef struct {
	GPIO_TypeDef *in1_port; uint8_t in1_pin;
	GPIO_TypeDef *in2_port; uint8_t in2_pin;
	volatile uint32_t *ccr;
	uint8_t invert;
} MotorCh_t;

enum {
	M_1A,
	M_1B,
	M_2A,
	M_2B,
	M_COUNT
};

static const MotorCh_t motors[M_COUNT] = {
	{GPIOC, 0, GPIOC, 1, &TIM3->CCR3, 0},
	{GPIOC, 2, GPIOC, 3, &TIM3->CCR4, 0},
	{GPIOC, 10, GPIOC, 11, &TIM4->CCR3, 0},
	{GPIOC, 12, GPIOD, 2, &TIM4->CCR4, 0},
};

static const uint8_t RIGHT_SIDE[2] = {M_1A, M_1B};
static const uint8_t LEFT_SIDE[2] = {M_2A, M_2B};

static volatile uint8_t g_speed = DEFAULT_SPEED;

// Driver

static inline void pin_write(GPIO_TypeDef *port, uint8_t pin, uint8_t level) {
	port->BSRR = level ? (1<<pin) : (1<<(pin+16));
}

static void gpio_output(GPIO_TypeDef *p, uint8_t pin) {
	p->MODER = (p->MODER & ~(3<<(pin*2))) | (1<<(pin*2));
}

static void gpio_aternate_func(GPIO_TypeDef *p, uint8_t pin, uint8_t af) {
	uint8_t idx = pin >> 3;
	uint8_t sh = ((pin & 7)*4);
	p->MODER = (p->MODER & ~(3<<(pin*2))) | (2<<(pin*2));
	p->AFR[idx] = (p->AFR[idx] & ~(0xF<<sh)) | ((uint32_t)af<<sh);
}


static void pwm_tim_init(TIM_TypeDef *t) {
	t->PSC = 0;
	t->ARR = PWM_ARR;
	t->CCR3 = 0;
	t->CCR4 = 0;
	t->CCMR2 = (6<<TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE | (6<<TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
	t->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;
	t->CR1 = TIM_CR1_ARPE;
	t->EGR = TIM_EGR_UG;
	t->CR1 |= TIM_CR1_CEN;
}

static void Motor_Drive(uint8_t idx, int8_t dir) {
	const MotorCh_t *m = &motors[idx];
	if(m->invert) dir = (int8_t)-dir;

	if(dir==0) {
		*m->ccr = 0;
		pin_write(m->in1_port, m->in1_pin, MOTOR_STOP);
		pin_write(m->in2_port, m->in2_pin, MOTOR_STOP);
		return;
	}

	pin_write(m->in1_port, m->in1_pin, dir>0);
	pin_write(m->in2_port, m->in2_pin, dir<0);
	*m->ccr = ((PWM_ARR + 1) * g_speed) / 100;
}

static void Drive(int8_t left, int8_t right) {
	for(uint8_t i = 0; i < 2; i++) {
		Motor_Drive(RIGHT_SIDE[i], right);
		Motor_Drive(LEFT_SIDE[i], left);
	}
}

// Init API
void Motor_Init(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
	(void)RCC->APB1ENR;

	for(uint8_t i = 0; i < M_COUNT; i++) {
		gpio_output(motors[i].in1_port, motors[i].in1_pin);
		gpio_output(motors[i].in2_port, motors[i].in2_pin);
	}

	gpio_aternate_func(GPIOC, 8, 2);
	gpio_aternate_func(GPIOC, 9, 2);
	gpio_aternate_func(GPIOB, 8, 2);
	gpio_aternate_func(GPIOB, 9, 2);

	pwm_tim_init(TIM3);
	pwm_tim_init(TIM4);

	Motor_Stop();
}
volatile void MotorSetSpeed(uint8_t percent) {
	g_speed = (percent > 100) ? 100 : percent;
}

uint8_t Motor_GetSpeed(void) {
	return g_speed;
}

void Motor_Forward(void) {
	Drive(+1, +1);
}

void Motor_Backward(void) {
	Drive(-1, -1);
}

void Motor_Left(void) {
	Drive(+1, -1);
}

void Motor_Right(void) {
	Drive(-1, +1);
}

void Motor_Stop(void) {
	Drive(0, 0);
}
