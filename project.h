#define PROJECT_H_

#include "LPC802.h"
#include <time.h>

//Define the ports for the mux switch
#define S0 (1UL << 12)
#define S1 (1UL << 7)
#define S2 (1UL << 1)

//Define the digits to the GPIO ports
#define DIGIT_0 (1UL << 4)
#define DIGIT_1 (1UL << 0)
#define DIGIT_2 (1UL << 11)
#define DIGIT_3 (1UL << 14)
#define DIGIT_4 (1UL << 13)

//wakeup timer reload value
#define WKT_RELOAD (100)

//button to turn on the system
#define BUTTON_USER1 (8)

//Relay signal
#define RELAY (1UL << 17)

//I2C defines
#define LPC_I2C0BUFFERSize (35)
#define LPC_I2C0BAUDRate (100000)// 100kHz
volatile uint8_t g_I2C0DataBuf[LPC_I2C0BUFFERSize];
volatile uint8_t g_I2C0DataCnt;
#define LM75_I2CAddress (0x4C)
// Define values for I2C registers that aren't in the header file.
// Table 195 of LPC802 User Manual
#define MSTCTL_CONTINUE (1UL << 0) // Bit 0 of MSTCTL set ("Main" or "Primary")
#define MSTCTL_START (1UL << 1) // Bit 1 of MSTCTL set ("Main" or "Primary")
#define MSTCTL_STOP (1UL << 2) // Bit 2 of MSTTCL set ("Main" or "Primary")
#define CTL_SLVCONTINUE (1UL << 0) // Bit 0: Secondary level (SLV) Continue
#define CTL_SLVNACK (1UL << 1) // Bit 1: Secondary Level (SLV) Acknowledge

#define PRIMARY_STATE_MASK (0x7<<1) // bits 3:1 of STAT register for Main / Primary
#define I2C_STAT_MSTST_IDLE ((0b000)<<1) // Main Idle: LPC802 user manual table 187
#define I2C_STAT_MSTST_RXRDY ((0b001)<<1) // Main Receive Ready " "
#define I2C_STAT_MSTST_TXRDY ((0b010)<<1) // Main Transmit Ready " "
#define I2C_STAT_MSTST_NACK_ADD ((0b011)<<1)// Main Ack Add " "
#define I2C_STAT_MSTST_NACK_DATA ((0b100)<<1)// Main Ack signal data ” ”

// What's your PWM Period?  Take your FRO freq. and divide by number of times per second you
// want the PWM to work.  So 24KHz is 12000000/24000 = 500.
#define CTIMER_PERIOD_TICKS (500) // 500 yields 24 KHz @ 12MHz FRO (i.e. 12M/24k)
#define PWM_PERCENT (0.80)// value b/w 0 and 1.  10% is 0.1, 80% is 0.8
#define CTIMER_MATCH (CTIMER_PERIOD_TICKS*(1-PWM_PERCENT))
#define OUTPUT_GPIO_FOR_PWM (0x9) // GPIO 9 for PWM signal

void WaitI2CPrimaryState(I2C_Type *ptr_LPC_I2C, uint32_t state);
void I2C_PrimarySetBaudRate(uint32_t baudRate_Bps, uint32_t srcClock_Hz);
void WKT_Config(void);
void SWM_init();
void lm75_i2c_init(void);
void lm75_read(float *data, uint8_t *orgdatabuffer);
void System_Handler();

//seven segment digits
int digit_0_num = 0;
int digit_1_num = 0;
int digit_2_num = 0;
int digit_3_num = 0;
int digit_4_num = 0;
int system_state = 0;

//fan speed change
int fanspeed = 0;

////config wakeup timer for seven segment
//void WKT_init() {
////setup 0: disable interrupts
//	__disable_irq();
//	NVIC_DisableIRQ(WKT_IRQn);
//// --------------------------------------------------------
//// Step 1: turn on the Wake-up Timer (WKT) enabling clock.
//	SYSCON->SYSAHBCLKCTRL0 |= (SYSCON_SYSAHBCLKCTRL0_WKT_MASK);
//// --------------------------------------------------------
//// Step 2: Turn on the Low Power Oscillator's power supply.
//// bit 6 to 0 in order to turn it ON. (Active low)
//	SYSCON->PDRUNCFG &= ~(SYSCON_PDRUNCFG_LPOSC_PD_MASK);
//
//	//--------------------------------------------------------
//// Step 3: Enable the LPOSC clock to the WKT. (Bit 1 to 1)
//	SYSCON->LPOSCCLKEN |= (SYSCON_LPOSCCLKEN_WKT_MASK);
//// --------------------------------------------------------
//// Step 4: Reset the WKT.
//// Set bit 9 to 0: assert (i.e. "make") the WKT reset
//// Set bit 9 to 1: clear the WKT reset (i.e. "remove" reset)
//	SYSCON->PRESETCTRL0 &= ~(SYSCON_PRESETCTRL0_WKT_RST_N_MASK); // Reset the WKT
//	SYSCON->PRESETCTRL0 |= (SYSCON_PRESETCTRL0_WKT_RST_N_MASK); // clear the reset.
//// --------------------------------------------------------
//// Step 5: Load the timer
//// --------------------------------------------------------
//// Select the LPOSC as the WKT clock source (0b0001)
//// Bit 0 (CLKSEL) to 1 for Low Power Clock
//// Bit 1 (ALARMFLAG) read flag. If 1, an interrupt has happened.
//// if 0, then no timeout.
//// Bit 2 (CLEARCTR). Set to 1 to clear the counter.
//// Bit 3 (SEL_EXTCLK). Set to 0 for internal clock source.
//	WKT->CTRL = WKT_CTRL_CLKSEL_MASK; // (Choose Low Power Clock using Bit 0)
//// load the timer count-down value. (The "Timeout value")
//	WKT->COUNT = WKT_RELOAD; // Start the WKT, counts down
//// WKT_RELOAD clocks then interrupts
//// Enable the IRQ
//	NVIC_EnableIRQ(WKT_IRQn); // Enable the WKT interrupt in the NVIC
//	__enable_irq();
//}

void SysTick_init() {
	__disable_irq(); // turn off globally
	NVIC_DisableIRQ(SysTick_IRQn); // turn off the SysTick interrupt.
	// ------------------------------------------------------------
	// Step 1: Choose & set main clock
	// ----------------------- Begin Core Clock Select -----------------------
	// Specify that we will use the Free-Running Oscillator
	// Set the main clock to be the FRO
	// 0x0 is FRO; 0x1 is external clock ; 0x2 is Low pwr osc.; 0x3 is FRO_DIV
	// Place in bits 1:0 of MAINCLKSEL.
	SYSCON->MAINCLKSEL = (0x0 << SYSCON_MAINCLKSEL_SEL_SHIFT); // FRO confirmed.
	// Update the Main Clock
	// Step 1. write 0 to bit 0 of this register
	// Step 2. write 1 to bit 0 this register
	SYSCON->MAINCLKUEN &= ~(0x1); // step 1. (Sec. 6.6.4 of manual)
	SYSCON->MAINCLKUEN |= 0x1; // step 2. (Sec. 6.6.4 of manual)
	// Set the FRO frequency (clock_config.h in SDK)
	// For FRO at 9MHz: BOARD_BootClockFRO18M();
	// 12MHz: BOARD_BootClockFRO24M();
	// 15MHz: BOARD_BootClockFRO30M();
	// See: Section 7.4 User Manual
	// This is more complete than just using set_fro_frequency(24000); for 12 MHz
	BOARD_BootClockFRO24M(); // 30M, 24M or 18M for 15MHz, 12MHz or 9MHz

	__enable_irq(); // turn on globally
	NVIC_EnableIRQ(SysTick_IRQn); // turn on the SysTick interrupt.

}

void GPIO_init() {
	__disable_irq();
	NVIC_DisableIRQ(PIN_INT0_IRQn);
	// setting up GPIO pins and interrupt
	SYSCON->SYSAHBCLKCTRL0 |= ( SYSCON_SYSAHBCLKCTRL0_GPIO0_MASK | // GPIO is on
			SYSCON_SYSAHBCLKCTRL0_GPIO_INT_MASK); // GPIO Interrupt is on
	// Put 0 in the GPIO and GPIO Interrupt reset bit to reset it.
	// Then put a 1 in the GPIO and GPIO Interrupt reset bit to allow both to operate.
	// manual: Section 6.6.10
	SYSCON->PRESETCTRL0 &= ~(SYSCON_PRESETCTRL0_GPIO0_RST_N_MASK |
	SYSCON_PRESETCTRL0_GPIOINT_RST_N_MASK);	// reset GPIO and GPIO Interrupt (bit=0)
	SYSCON->PRESETCTRL0 |= (SYSCON_PRESETCTRL0_GPIO0_RST_N_MASK |
	SYSCON_PRESETCTRL0_GPIOINT_RST_N_MASK);	// clear reset (bit=1)

	//disable interupts

	//input as button 8 on the lpc802 board
	GPIO->DIRCLR[0] = (1UL << BUTTON_USER1);

	// Set up GPIO IRQ: interrupt channel 0 (PINTSEL0) to GPIO 8
	SYSCON->PINTSEL[0] = BUTTON_USER1; // PINTSEL0 is P0_8
	// Configure the Pin interrupt mode register (a.k.a ISEL) for edge-sensitive
	// on PINTSEL0. 0 is edge sensitive. 1 is level sensitive.
	PINT->ISEL = 0x00; // channel 0 bit is 0: is edge sensitive (so are the other channels)
	// Use IENR or IENF (or S/CIENF or S/CIENR) to set edge type
	// Configure Chan 0 for only falling edge detection (no rising edge detection)
	PINT->CIENR = 0b00000001; // bit 0 is 1: disable channel 0 IRQ for rising edge
	PINT->SIENF = 0b00000001; // bit 0 is 1: enable channel 0 IRQ for falling edge
	// Remove any pending or left-over interrupt flags
	PINT->IST = 0xFF; // each bit set to 1 removes any pending flag.

	NVIC_EnableIRQ(PIN_INT0_IRQn); // GPIO interrupt
	__enable_irq();

	//-----------------seven segment display setup

	GPIO->DIRCLR[0] = (DIGIT_0 | DIGIT_1 | DIGIT_2 | DIGIT_3 | DIGIT_4); //setup for digits
	GPIO->CLR[0] = (DIGIT_0 | DIGIT_1 | DIGIT_2 | DIGIT_3 | DIGIT_4);
	GPIO->DIRSET[0] = (DIGIT_0 | DIGIT_1 | DIGIT_2 | DIGIT_3 | DIGIT_4);

	GPIO->DIRCLR[0] = (S0 | S1 | S2); //settup for MUX
	GPIO->CLR[0] = (S0 | S1 | S2);
	GPIO->DIRSET[0] = (S0 | S1 | S2);

	GPIO->DIRCLR[0] = RELAY; //setup for Relay
	GPIO->CLR[0] = RELAY;
	GPIO->DIRSET[0] = RELAY;
}

void SWM_init() {
	// enable switch matrix

	SYSCON->SYSAHBCLKCTRL0 |= (SYSCON_SYSAHBCLKCTRL0_SWM_MASK);
// Set switch matrix
// ---> PINASSIGN4 T0-MAT0 is (bits 7:0) is 0x9 (OUTPUT_GPIO_FOR_PWM)
// Connect CTimer0 channel 0 to an external LED pin
	SWM0->PINASSIGN.PINASSIGN4 &= ~(0xff); // clear the bottom 8 bits
	SWM0->PINASSIGN.PINASSIGN4 |= (OUTPUT_GPIO_FOR_PWM); // put 0x17 in the bottom 8 bits.

// Enable CTIMER clock
	SYSCON->SYSAHBCLKCTRL0 |= (SYSCON_SYSAHBCLKCTRL0_CTIMER0_MASK);
// Reset the CTIMER module
// 1. Assert CTIMER-RST-N
// 2. Clear CTIMER-RST-N
	SYSCON->PRESETCTRL0 &= ~(SYSCON_PRESETCTRL0_CTIMER0_RST_N_MASK); // Reset
	SYSCON->PRESETCTRL0 |= (SYSCON_PRESETCTRL0_CTIMER0_RST_N_MASK); // clear the reset.

// Enable PWM mode on the channel 0
// **************************************
// 1. EMR bit 0 set to 1  (External Match Register)
// 2. PWMEN0 (PWMC bit 0) set to "PWM"
// ****************************************
	CTIMER0->EMR |= CTIMER_EMR_EM0_MASK; // (rule complicated)
	CTIMER0->PWMC |= CTIMER_PWMC_PWMEN0_MASK; // 0 is match mode; 1 is PWM mode.

// Clear all the Channel 0 bits in the MCR
	CTIMER0->MCR &= ~(CTIMER_MCR_MR0R_MASK | CTIMER_MCR_MR0S_MASK
			| CTIMER_MCR_MR0I_MASK);
// Reset the counter when match on channel 3
// ***********************************
// MCR bit MR3R gets set to 1
// ***********************************
	CTIMER0->MCR |= CTIMER_MCR_MR3R_MASK; // MR3R bit to 1
// Match on channel 3 will define the PWM period
// If we use the 12MHz internal clock, then 12000000 will make it 1 Hz.
// ************************************
	CTIMER0->MR[3] = CTIMER_PERIOD_TICKS; //
// This will define the PWM pulse off time.
// If this is 75% of MR[3] value you'll get 25 Duty Cycle.
// Make this freq * (1-PWM_DUTY_FACTOR).
	CTIMER0->MR[0] = CTIMER_MATCH; // DUTY FACTOR is 0.75

// Start the timer:
// **************************************************
// 1. CTIMER0->TCR CEN bit (bit 0) got set to ENABLED
// 2. The timer counter (TC) will contain values... CTIMER0->TC
// **************************************************
	CTIMER0->TCR |= CTIMER_TCR_CEN_MASK;  // 0 is disabled. 1 is enabled.
// at this point, the TCR ENable bit gets set to 1
// and then, automatically, we should see the timer TCVAL value change.
// that should confirm that the timer is actually running.
}

void lm75_i2c_init(void) {
// ------------------------------------------------------------
// Step 1: Connect I2C module to outer pins via Switch Matrix
// PIO0_16 is connected to the SCL line
// PIO0_10 is connected to the SDA line
// PINASSIGN5 bits 15:8 are for SCL. Therefore fill with value 16
// PINASSIGN5 bits 7:0 are for SDA. Therefore fill with value 10
//
// enable switch matrix.
	SYSCON->SYSAHBCLKCTRL0 |= (SYSCON_SYSAHBCLKCTRL0_SWM_MASK);
// Set switch matrix
// PINASSIGN5: clear bits 15:0 to permit the two i2c lines to be assigned.
	SWM0->PINASSIGN.PINASSIGN5 &= ~(SWM_PINASSIGN5_I2C0_SCL_IO_MASK |
	SWM_PINASSIGN5_I2C0_SDA_IO_MASK); // clear 15:0
	SWM0->PINASSIGN.PINASSIGN5 |= ((16 << SWM_PINASSIGN5_I2C0_SCL_IO_SHIFT) | // Put 16 in bits 15:8
			(10 << SWM_PINASSIGN5_I2C0_SDA_IO_SHIFT)); // put 10 in bits 7:0
// PINASSIGN5 should be 0xffff100a after this.
// disable the switch matrix
	SYSCON->SYSAHBCLKCTRL0 &= ~(SYSCON_SYSAHBCLKCTRL0_SWM_MASK);
// ---------------- End of Switch Matrix code -----------------------------------

// ------------------------------------------------------------
// Step 2: Turn on the I2C module via the SYSCON clock enable pin
// ------------------------------------------------------------
// debug: just i2c
	SYSCON->SYSAHBCLKCTRL0 |= (SYSCON_SYSAHBCLKCTRL0_I2C0_MASK); // I2C is on
// debug: in the demo code they don't seem to reset the i2c clock.
// debug: just i2c
// Put 0 in the GPIO, GPIO Interrupt and I2C reset bit to reset it.
// Then put a 1 in the GPIO, GPIO Interrupt and I2C reset bit to allow them to operate.
// manual: Section 6.6.10
	SYSCON->PRESETCTRL0 &= ~(SYSCON_PRESETCTRL0_I2C0_RST_N_MASK); // reset I2C(bit = 0)
	SYSCON->PRESETCTRL0 |= (SYSCON_PRESETCTRL0_I2C0_RST_N_MASK); // remove i2C reset (bit = 1)

// ------------------------------------------------------------
// Step 3: Choose the source of the I2C timing clock
// see: Fig 7 of User Manual
// ------------------------------------------------------------
// Provide main_clk as function clock to I2C0
// bits 2:0 are 0x1.
// FRO is 0b000; main clock is 0b001, frg0clk is 0b010, fro_div is 0b100.
// Set i2c to FRO (12 MHz)
// (only bits 2:0 are used; other 29 bits are ignored)
	SYSCON->I2C0CLKSEL = 0b000; // put 000 in bits 2:1.
// confirmed in reg view that this is FRO.

// ------------------------------------------------------------
// Step 4: enable primary (but not slave) functionality in the i2c module.
//
// ------------------------------------------------------------
// Configure the I2C0 CFG register:
// Primary enable = true
// Secondary enable = true
// Monitor enable = false
// Time-out enable = false
// Monitor function clock stretching = false
//
// Only config I2C0 as a primary
	I2C0->CFG = (I2C_CFG_MSTEN_MASK); // only as primary
// Comment out for now: therefore, no interruptions.
// Enable the I2C0 "secondary" pending interrupt
// I2C0->INTENSET = I2C_INTENSET_SLVPENDINGEN_MASK; // STAT_SLVPEND;

// ------------------------------------------------------------
// Step 5: set the i2c clock rate. (20Hz to 400kHz is typical)
// I2C_PrimarySetBaudRate() function is helpful.
// ------------------------------------------------------------
// Set i2C bps to 100kHz assuming an input clock of 12MHz
	I2C_PrimarySetBaudRate(100000, 12000000);
// after this, CLKDIV is 0x9 and MSTTIME is 0x44.
// MSTTIME = 0x44 means: MSTSCLLOW [2:0] is CLOCKS_6
// MSTCLHIGH [6:4] is CLOCKS_6
}

void lm75_read(float *data, uint8_t *orgdatabuffer) {
	uint8_t LM75_Buf[2];

	WaitI2CPrimaryState(I2C0, I2C_STAT_MSTST_IDLE); // Wait for the Primary's state to be idle
	I2C0->MSTDAT = (LM75_I2CAddress << 1) | 0; // Address with 0 for RWn bit (WRITE)
	I2C0->MSTCTL = MSTCTL_START; // Start the transaction by setting the

// MSTSTART bit to 1 in the Primary control register
	WaitI2CPrimaryState(I2C0, I2C_STAT_MSTST_TXRDY); // Wait for the address to be ACK'd
	I2C0->MSTDAT = 0x00;
	I2C0->MSTCTL = MSTCTL_CONTINUE;

	WaitI2CPrimaryState(I2C0, I2C_STAT_MSTST_TXRDY); // Wait for the address to be ACK'd
	I2C0->MSTDAT = (LM75_I2CAddress << 1) | 1; // Address with 1 for RWn bit (WRITE)
	I2C0->MSTCTL = MSTCTL_START; // Start the transaction by setting the

// MSTSTART bit to 1 in the Primary control register.
	while ((I2C0->STAT & PRIMARY_STATE_MASK) != I2C_STAT_MSTST_RXRDY) {
// just spin... (wait)
	}

	LM75_Buf[0] = I2C0->MSTDAT;
	I2C0->MSTCTL = MSTCTL_CONTINUE;
// Wait for the address to be ACK’d
	WaitI2CPrimaryState(I2C0, I2C_STAT_MSTST_RXRDY);
// PRIMARY_STATE_MASK is (0x7<<1)
	while ((I2C0->STAT & PRIMARY_STATE_MASK) != I2C_STAT_MSTST_RXRDY) {
// just spin... (wait)
	}
	LM75_Buf[1] = I2C0->MSTDAT;
	I2C0->MSTCTL = MSTCTL_STOP;

	while ((I2C0->STAT & PRIMARY_STATE_MASK) != I2C_STAT_MSTST_IDLE) {
// just spin...
	}
// Convert the buffer into a single variable
// accessible outside this function via the *data pointer.
	if ((LM75_Buf[0] & 0x80) == 0x00) {
		*data = ((float) (((LM75_Buf[0] << 8) + LM75_Buf[1]) >> 5) * 0.125);
	} else {
		*data = 0x800 - ((LM75_Buf[0] << 8) + (LM75_Buf[1] >> 5));
		*data = -(((float) (*data)) * 0.125);
	}
}

void WaitI2CPrimaryState(I2C_Type *ptr_LPC_I2C, uint32_t state) {
	// Check the Primary Pending bit (bit 0 of the i2c stat register)
	// Wait for MSTPENDING bit set in STAT register
	while (!(ptr_LPC_I2C->STAT & I2C_STAT_MSTPENDING_MASK))
		; // Wait
	// Check to see that the state is correct.
	// if it is not, then turn on PIO0_9 to indicate a problem
	// Primary's state is in bits 3:1. PRIMARY_STATE_MASK is (0x7<<1)
	if ((ptr_LPC_I2C->STAT & PRIMARY_STATE_MASK) != state) // If primary state mismatch...
			{
		GPIO->DIRCLR[0] = (1UL << 9); // turn on LED on PIO0_9 (LED_USER2)
		while (1)
			; // die here and debug the problem
	}
	return; // If no mismatch, return
}

void I2C_PrimarySetBaudRate(uint32_t baudRate_Bps, uint32_t srcClock_Hz) {
	uint32_t scl, divider;
	uint32_t best_scl, best_div;
	uint32_t err, best_err;
	best_err = 0;

	for (scl = 9; scl >= 2; scl--) {
		/* calculated ideal divider value for given scl */
		divider = srcClock_Hz / (baudRate_Bps * scl * 2u);
		/* adjust it if it is out of range */
		divider = (divider > 0x10000u) ? 0x10000 : divider;
		/* calculate error */
		err = srcClock_Hz - (baudRate_Bps * scl * 2u * divider);
		if ((err < best_err) || (best_err == 0)) {
			best_div = divider;
			best_scl = scl;
			best_err = err;
		}
		if ((err == 0) || (divider >= 0x10000u)) {
			/* either exact value was found
			 or divider is at its max (it would even greater in the next iteration for sure) */
			break;
		}
	}
// Assign Clock Divider value, using macro included in LPC802.h
	I2C0->CLKDIV = I2C_CLKDIV_DIVVAL(best_div - 1);
// Assign Primary timing configuration, using two macros include in LPC802.h
	I2C0->MSTTIME =
	I2C_MSTTIME_MSTSCLLOW(
			best_scl - 2u) | I2C_MSTTIME_MSTSCLHIGH(best_scl - 2u);
}

void display_digit(int number, int digit_x) {

	switch (number) {
	case (0):
		GPIO->SET[0] = digit_x; //turn on the digit
		GPIO->SET[0] = S1; // channel_2 (led_f)
		GPIO->SET[0] = S2; // channel_6 (led_b)
		GPIO->SET[0] = S0; //channel_7 (led_a)
		GPIO->NOT[0] = (S0 | S1); // channel_4 (led_d)
		GPIO->NOT[0] = (S0 | S1 | S2); // channel_3 (led_e)
		GPIO->NOT[0] = (S1 | S2); //channel_5 (led_c)
		GPIO->CLR[0] = digit_x;  // turn off digit
		GPIO->CLR[0] = (S0 | S1 | S2); // clear all leds
		break;
	case (1):
		GPIO->SET[0] = digit_x;
		GPIO->SET[0] = (S1 | S2); //channel_6 (led_b)
		GPIO->NOT[0] = (S0 | S1); //channel_5 (led_c)
		GPIO->CLR[0] = digit_x;  // turn off digit
		GPIO->CLR[0] = (S0 | S1 | S2); // clear all leds
		break;
	case (2):
		GPIO->SET[0] = digit_x;
		GPIO->SET[0] = S0; // channel_1 (led_g)
		GPIO->NOT[0] = (S0 | S2); // channel_4 (led_d)
		GPIO->NOT[0] = (S0 | S1 | S2); //channel_3 (led_e)
		GPIO->SET[0] = S2; //channel_7 (led_a);
		GPIO->NOT[0] = S0; //channel_6 (led_b)
		GPIO->CLR[0] = digit_x;  // turn off digit
		GPIO->CLR[0] = (S0 | S1 | S2); // clear all leds
		break;
	case (3):
		GPIO->SET[0] = digit_x;
		GPIO->SET[0] = S0;  // channel_1 (led_g)
		GPIO->SET[0] = S2; // channel_5 (led_c)
		GPIO->SET[0] = S1; //channel_7 (led_a)
		GPIO->NOT[0] = S0; // channel_6 (led_b)
		GPIO->NOT[0] = S1; // channel_4 (led_d)
		GPIO->CLR[0] = digit_x;  // tturn off digit
		GPIO->CLR[0] = (S0 | S1 | S2); // clear all leds
		break;
	case (4):
		GPIO->SET[0] = digit_x;
		GPIO->SET[0] = S0; // channel_1 (led_g)
		GPIO->NOT[0] = (S0 | S1); //channel_2 (led_f)
		GPIO->NOT[0] = (S0 | S1 | S2); // channel_5 (led_c)
		GPIO->NOT[0] = (S0 | S1); // channel_6 (led_b)
		GPIO->CLR[0] = digit_x;  // turn off digit
		GPIO->CLR[0] = (S0 | S1 | S2); //clear all leds
		break;
	case (5):
		GPIO->SET[0] = digit_x; // 0
		GPIO->SET[0] = S0; // channel_1 (led_g)
		GPIO->SET[0] = S2; //channel_5 (led_c)
		GPIO->SET[0] = S1; //channel_7 (led_a)
		GPIO->NOT[0] = (S0 | S2); // channel_2 (led_f)
		GPIO->NOT[0] = (S1 | S2); // channel_4 (led_d)
		GPIO->CLR[0] = digit_x;  // turn off digit
		GPIO->CLR[0] = (S0 | S1 | S2); // clear all leds
		break;
	case (6):
		GPIO->SET[0] = digit_x; // 0
		GPIO->SET[0] = S0; // channel_1 (led_g)
		GPIO->SET[0] = S1; // channel_3 (led_e)
		GPIO->SET[0] = S2; //channel_7 (led_a)
		GPIO->NOT[0] = (S0 | S2); // channel_2 (led_f)
		GPIO->NOT[0] = (S0 | S1 | S2); // channel_5 (led_c)
		GPIO->NOT[0] = S0; //channel_4 (led_d)
		GPIO->CLR[0] = digit_x;  // turn off digit
		GPIO->CLR[0] = (S0 | S1 | S2); // clear all leds
		break;
	case (7):
		GPIO->SET[0] = digit_x;
		GPIO->SET[0] = (S0 | S1 | S2); // channel_7 (led_a)
		GPIO->NOT[0] = S0; // channel_6 (led_b)
		GPIO->NOT[0] = (S0 | S1); // channe_5 (led_c)
		GPIO->CLR[0] = digit_x;  // turn off digit
		GPIO->CLR[0] = (S0 | S1 | S2); // clear all leds
		break;
	case (8):
		GPIO->SET[0] = digit_x;
		GPIO->SET[0] = S1; // channel_2 (led_f)
		GPIO->SET[0] = S2; // channel_6 (led_b)
		GPIO->SET[0] = S0; //channel_7 (led_a)
		GPIO->NOT[0] = (S0 | S1); // channel_4 (led_d)
		GPIO->NOT[0] = (S0 | S1 | S2); // channel_3 (led_e)
		GPIO->NOT[0] = (S1 | S2); //channel_5 (led_c)
		GPIO->NOT[0] = S2; //channel_1 (led_g)
		GPIO->CLR[0] = digit_x;  // turn off digit
		GPIO->CLR[0] = (S0 | S1 | S2); // clear all leds
		break;
	case (9):
		GPIO->SET[0] = digit_x;
		GPIO->SET[0] = S1; // channel_2 (led_f)
		GPIO->SET[0] = S2; // channel_6 (led_b)
		GPIO->SET[0] = S0; //channel_7 (led_a)
		GPIO->NOT[0] = (S0 | S1); // channel_4 (led_d)
		GPIO->NOT[0] = (S0 | S2); //channel_1 (led_g)
		GPIO->NOT[0] = S2;
		GPIO->CLR[0] = digit_x;  // turn off digit
		GPIO->CLR[0] = (S0 | S1 | S2); // clear all leds
		break;
	case (-2):
		//DIsplay F
		GPIO->SET[0] = digit_x;
		GPIO->SET[0] = (S0 | S1 | S2); //channel_7 (led_a)
		GPIO->NOT[0] = (S0 | S2); //channel_3 (led_e)
		GPIO->NOT[0] = S0; //channel_2 (led_f)
		GPIO->CLR[0] = digit_x;
		GPIO->CLR[0] = (S0 | S1 | S2);
	case (-1):
		//Display -
		GPIO->SET[0] = digit_x;
		GPIO->SET[0] = S0; // channel_1 (led_g)
		GPIO->CLR[0] = digit_x;  // turn off digit
		GPIO->CLR[0] = (S0 | S1 | S2); // clear all leds
		break;
	}

}
