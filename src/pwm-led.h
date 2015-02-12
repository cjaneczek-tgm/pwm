/*
 Name of file		:	pwm-led.h
 Author				:	Christian Janeczek <cjaneczek@student.tgm.ac.at>
 Version			:	2015-02-12
 Description		:	This headfile defines any output pins and bits to enable working with LEDs and PWM
*/

#define LED_RED		GPIO_PIN_1
#define LED_BLUE	GPIO_PIN_2
#define LED_GREEN	GPIO_PIN_3

#define RED_OUT		PWM_OUT_5
#define GREEN_OUT	PWM_OUT_7
#define BLUE_OUT	PWM_OUT_6

#define RED_OUT_BIT		PWM_OUT_5_BIT
#define GREEN_OUT_BIT	PWM_OUT_7_BIT
#define BLUE_OUT_BIT	PWM_OUT_6_BIT
