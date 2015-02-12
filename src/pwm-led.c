/*
 Name of file		:	pwm-led.c
 Author				:	Christian Janeczek <cjaneczek@student.tgm.ac.at>
 Version			:	2015-02-12
 Description		:	An application that dims the LED by changing the duty cycle via Pulse-Width-Modulation
 						You can change the duty cycle by pressing the SW_0. The available modi are 25%, 50%, 75% and 100%.
*/

#include "pwm-led.h"

#include <stdint.h>
#include <stdbool.h>

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"

#include "driverlib/rom_map.h"
#include "driverlib/interrupt.h"

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_ints.h"

#include "drivers/buttons.h"

#include "driverlib/uart.h"
#include "utils/uartstdio.h"

void InitPWM(void);
void InitISR(void);
void IntButtonHandler(void);
void InitConsole(void);

uint32_t cycle = 0;
uint32_t state = 0;
uint32_t duty = 250; //defining a duty cycle of 1.5ms (pulse-width)
uint32_t period = 5000; //20ms (16Mhz / 64pwm_divider / 50)
uint32_t out = RED_OUT, outbit = RED_OUT_BIT;

int main(void)
{

	// Setup the clock
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

	// Setup the PWM
	InitPWM();

	// Initialize the console output
	InitConsole();

	// Initialize the InterruptServiceRoutine
	InitISR();

	while (1)
	{ // loop endless
	}
}

/**
 * Initializing the Pulse-Width-Modulation
 *
 */
void InitPWM(void)
{

	/*
	 * Set up the PWM clock
	 */
	// SYSCTL_PWMDIV_1
	// SYSCTL_PWMDIV_32
	// SYSCTL_PWMDIV_64
	ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

	/*
	 * Enable the port for the leds
	 */
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	/*
	 * Enable the PWM1 Genatator
	 */
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

	/*
	 * Configure the led ports for the PWM output signal
	 */
	GPIOPinConfigure(GPIO_PF1_M1PWM5);
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
	GPIOPinTypePWM(GPIO_PORTF_BASE, LED_RED | LED_GREEN | LED_BLUE ); // set the output port

	//Configure PWM generator Options
    //PWM_GEN_2 Covers M1PWM4 and M1PWM5
    //PWM_GEN_3 Covers M1PWM6 and M1PWM7
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN);

    //Set the period (expressed in clock ticks)
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, period);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, period);

    // Enable the PWM generator
    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

}

/**
 * Initializing the Interrupt-Service-Routine
 *
 */
void InitISR(void)
{

	ROM_SysCtlPeripheralEnable(BUTTONS_GPIO_PERIPH);
	HWREG(BUTTONS_GPIO_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
  	HWREG(BUTTONS_GPIO_BASE + GPIO_O_CR) |= 0x01;
  	HWREG(BUTTONS_GPIO_BASE + GPIO_O_LOCK) = 0;
	ROM_GPIODirModeSet(BUTTONS_GPIO_BASE, ALL_BUTTONS, GPIO_DIR_MODE_IN);
	ROM_GPIOPadConfigSet(BUTTONS_GPIO_BASE, ALL_BUTTONS, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

  	ROM_GPIOIntTypeSet(BUTTONS_GPIO_BASE, ALL_BUTTONS, GPIO_FALLING_EDGE);
  	GPIOIntEnable(BUTTONS_GPIO_BASE, ALL_BUTTONS);
  	ROM_IntEnable(INT_GPIOF);
  	ROM_IntMasterEnable();

}

/**
 * Initializing UART to accomplish console outputs for debugging purposes
 *
 */
void InitConsole(void)
{

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	
	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	ROM_UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
	UARTStdioConfig(0, 115200, 16000000);

	UARTprintf("UART is running ... \n");

}

/**
 * Definition of the ButtonHandler which reacts to one of the two switches being pressed.
 *
 * If any of the switches are pressed, the duty cycle will be adjusted by using PWM.
 *
 */
void IntButtonHandler(void)
{

	GPIOIntClear(BUTTONS_GPIO_BASE, ALL_BUTTONS);
	PWMOutputState(PWM1_BASE, outbit, true); // enable the output state for the pwm signal

	switch(cycle)
	{
		case 0:
			cycle++;
			//setting the duty cycle on 25%
			PWMPulseWidthSet(PWM1_BASE, out, 1249);
			UARTprintf("Cycle: %i Intensity: 25%%\n", cycle);
			break;
		
		case 1:
			//setting the duty cycle on 50%
			PWMPulseWidthSet(PWM1_BASE, out, 2499);
			cycle++;
			UARTprintf("Cycle: %i Intensity: 50%%\n", cycle);
			break;

		case 2:
			cycle++;
			//setting the duty cycle on 75%
			PWMPulseWidthSet(PWM1_BASE, out, 3749);
			UARTprintf("Cycle: %i Intensity: 75%%\n", cycle);
			break;

		case 3:
			//setting the duty cycle on 100%
			PWMPulseWidthSet(PWM1_BASE, out, 4999);
			UARTprintf("Cycle: %i Intensity: 100%%\n", cycle);
			cycle = 0;
			//resetting the counter variable
			break;
	}

}

