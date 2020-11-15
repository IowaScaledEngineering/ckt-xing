#include <samd21.h>
#include <stdbool.h>
#include "bell.h"

// Various state machine timeouts

#define EAST_APPROACH_TIMEOUT             100
#define EAST_DEPARTURE_TIMEOUT             10
#define EAST_LOCKOUT_TIMER                100

#define WEST_APPROACH_TIMEOUT             100
#define WEST_DEPARTURE_TIMEOUT             10
#define WEST_LOCKOUT_TIMER                100

#define ISLAND_TIMER                       20


// I/O Ports definitions
#define PORTA     (0ul)
#define PORTB     (1ul)

#define PB08  8
#define PB10 10
#define PB11 11
#define PB22 22
#define PB23 23

#define PA02  2
#define PA15 15
#define PA16 16
#define PA17 17
#define PA18 18
#define PA19 19
#define PA20 20

#define IO_INPUT_AUX_IN_MASK     PORT_PB10
#define IO_INPUT_ACTV_MASK       PORT_PB11
#define IO_INPUT_WOCC_MASK       PORT_PB22
#define IO_INPUT_EOCC_MASK       PORT_PB23

#define IO_OUTPUT_AUX_OUT_MASK   PORT_PA13
#define IO_OUTPUT_GATE_A_MASK    PORT_PA21
#define IO_OUTPUT_GATE_B_MASK    PORT_PA22


// Constants for Clock Generators
#define GENERIC_CLOCK_GENERATOR_MAIN      (0u)
#define GENERIC_CLOCK_GENERATOR_XOSC32K   (1u)
#define GENERIC_CLOCK_GENERATOR_OSCULP32K (2u) /* Initialized at reset for WDT */
#define GENERIC_CLOCK_GENERATOR_OSC8M     (3u)
// Constants for Clock multiplexers
#define GENERIC_CLOCK_MULTIPLEXER_DFLL48M (0u)

// Constants for DFLL48M
#define MAIN_CLK_FREQ (48000000u)
#define EXT_32K_CLK_FREQ (32768u)

volatile uint32_t runFlags = 0;

#define RUN_SENSOR_READ 0x00000001

#define ON_DEBOUNCE_DEFAULT     1
#define OFF_DEBOUNCE_DEFAULT    4
#define   TMD26711_ADDR   0x39
#define   INFO_ADDR       0x20

#define   PROXIMITY_THRESHOLD     0x300
#define   SENSOR_ERROR_THRESHOLD  0
#define   PPULSE_DEFAULT          8

#define   ON_DEBOUNCE_DEFAULT   1

#define _BV(i)  (1<<(i))

void delay_us(uint16_t microseconds);
void delay_ms(uint32_t milliseconds);

#define LIGHT_LEFT_ON   0x01
#define LIGHT_RIGHT_ON  0x02
#define LIGHT_CONST_ON  0x04
#define LIGHT_RATE  0x800

volatile bool lightsOn = false;

void LightPWM(bool phase)
{
	static uint16_t leftLightPWMSetting = 0;
	static uint16_t rightLightPWMSetting = 0;
	static uint16_t constLightPWMSetting = 0;
	uint8_t lights = 0;

	if (lightsOn && phase)
		lights = LIGHT_LEFT_ON | LIGHT_CONST_ON;
	else if (lightsOn && !phase)
		lights = LIGHT_RIGHT_ON | LIGHT_CONST_ON;
	else
		lights = 0;

	// Do light things
	if (lights & LIGHT_LEFT_ON)
	{
		if (leftLightPWMSetting <= 0x8000)
			leftLightPWMSetting += LIGHT_RATE;

	} else {
		if (leftLightPWMSetting >= LIGHT_RATE)
			leftLightPWMSetting -= LIGHT_RATE;
		else 
			leftLightPWMSetting = 0;
	}

	// Do light things
	if (lights & LIGHT_RIGHT_ON)
	{
		if (rightLightPWMSetting <= 0x8000)
			rightLightPWMSetting += LIGHT_RATE;

	} else {
		if (rightLightPWMSetting >= LIGHT_RATE)
			rightLightPWMSetting -= LIGHT_RATE;
		else 
			rightLightPWMSetting = 0;
	}

	// Do light things
	if (lights & LIGHT_CONST_ON)
	{
		if (constLightPWMSetting <= 0x8000)
			constLightPWMSetting += LIGHT_RATE;

	} else {
		if (constLightPWMSetting >= LIGHT_RATE)
			constLightPWMSetting -= LIGHT_RATE;
		else 
			constLightPWMSetting = 0;
	}
	TCC0->CCB[0].reg = leftLightPWMSetting;
	while(TCC0->SYNCBUSY.bit.CC0);
	TCC0->CCB[1].reg = constLightPWMSetting;
	while(TCC0->SYNCBUSY.bit.CC1);
	TCC0->CCB[2].reg = rightLightPWMSetting;
	while(TCC0->SYNCBUSY.bit.CC2);

}

// Should be a 1ms ISR
void SysTick_Handler(void)
{
	static uint32_t ul_tickcount=0 ;	// Global state variable for tick count
	static bool phase = false;
	static uint32_t phaseTick = 0;

	ul_tickcount++ ;

	if (ul_tickcount % 10 == 0)
	{
		if (!lightsOn)
		{
			phaseTick = 0;
			phase = false;
		} else if (phaseTick++ % 80 == 0) {
			phase = !phase;
		}

		LightPWM(phase);
	}

	if (ul_tickcount % 100 == 0)
	{
		runFlags |= RUN_SENSOR_READ;
	}
}


void ClocksInit(void);

typedef struct
{
	uint32_t clock_A;
	uint32_t clock_B;
	uint32_t debounced_state;
} DebounceState;

void initDebounceState(DebounceState* d, uint32_t initialState)
{
	d->clock_A = d->clock_B = 0;
	d->debounced_state = initialState;
}

uint32_t debounce(uint32_t raw_inputs, DebounceState* d)
{
	uint32_t delta = raw_inputs ^ d->debounced_state;   //Find all of the changes
	uint32_t changes;

	d->clock_A ^= d->clock_B;                     //Increment the counters
	d->clock_B  = ~d->clock_B;

	d->clock_A &= delta;                       //Reset the counters if no changes
	d->clock_B &= delta;                       //were detected.

	changes = ~((~delta) | d->clock_A | d->clock_B);
	d->debounced_state ^= changes;
	return(changes & ~(d->debounced_state));
}


volatile int32_t volume = 0x2000;

void ADC_Setup()
{
	// Enable the peripheral bus interface clock
	PM->APBCMASK.bit.ADC_ = 1;

	// Send the ADC module 8MHz, which we'll cut down to 2MHz for operations
	GCLK_CLKCTRL_Type gclk_clkctrl = {
		.bit.WRTLOCK = 0,		/* Generic Clock is not locked from subsequent writes */
		.bit.CLKEN = 1,			/* Enable the Generic Clock */
		.bit.GEN = GENERIC_CLOCK_GENERATOR_OSC8M, 	/* Generic Clock Generator 3 (8MHz) is the source */
		.bit.ID = ADC_GCLK_ID			/* Generic Clock Multiplexer for ADC  */
	};
	// Write these settings
	GCLK->CLKCTRL.reg = gclk_clkctrl.reg;
	while (GCLK->STATUS.bit.SYNCBUSY);

	// Change ADC0 and ADC2 to analog inputs
	PORT->Group[PORTA].DIRCLR.reg = PORT_PA02;
	PORT->Group[PORTB].DIRCLR.reg = PORT_PB08;
	
	PORT->Group[PORTA].PINCFG[2].bit.PMUXEN = 1;  // Analog 0
	PORT->Group[PORTB].PINCFG[8].bit.PMUXEN = 1;  // Analog 1
	
	PORT->Group[PORTA].PMUX[1].bit.PMUXE = 1; // Analog is peripheral function B = 1
	PORT->Group[PORTB].PMUX[4].bit.PMUXE = 1; // Analog is peripheral function B = 1

	// Software reset
	ADC->CTRLA.bit.SWRST = 1;
	while (ADC->STATUS.bit.SYNCBUSY);
	
	// Load calibration constants
	uint32_t bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
	uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
	linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;

	// Wait for bus synchronization.
	while (ADC->STATUS.bit.SYNCBUSY);
	ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);

	// Set reference to 1/2 VCCANA
	ADC_REFCTRL_Type adc_refctrl = {
		.bit.REFCOMP = 1,
		.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val
	};
	ADC->REFCTRL.reg = adc_refctrl.reg;

	ADC_AVGCTRL_Type adc_avgctrl = {
		.bit.SAMPLENUM = ADC_AVGCTRL_SAMPLENUM_1024,
		.bit.ADJRES = 0x04 // keep the output as 12 bit
	};
	ADC->AVGCTRL.reg = adc_avgctrl.reg;

	ADC_CTRLB_Type adc_ctrlb = {
		.bit.DIFFMODE = 0,
		.bit.LEFTADJ = 0,
		.bit.FREERUN = 0,
		.bit.CORREN = 0,
		.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val,
		.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV4_Val
	};
	ADC->CTRLB.reg = adc_ctrlb.reg;
	while (ADC->STATUS.bit.SYNCBUSY);
	
	ADC_INPUTCTRL_Type adc_inputctrl = {
		.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_SCALEDIOVCC_Val,
		.bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val,
		.bit.INPUTSCAN = 0,
		.bit.INPUTOFFSET = 0,
		.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val
	};
	ADC->INPUTCTRL.reg = adc_inputctrl.reg;
	while (ADC->STATUS.bit.SYNCBUSY);


	ADC->DBGCTRL.bit.DBGRUN = 1;

	ADC_CTRLA_Type adc_ctrla = {
		.bit.RUNSTDBY = 0,
		.bit.ENABLE = 1,
	};
	ADC->CTRLA.reg = adc_ctrla.reg;
	while (ADC->STATUS.bit.SYNCBUSY);

	NVIC_SetPriority(ADC_IRQn, 3);	// Set interrupt priority to 3
	NVIC_EnableIRQ(ADC_IRQn);		// Enable ADC interrupt

	// Enable result ready interrupt
	ADC->INTENSET.bit.RESRDY = 1;

	// Start analog conversion
	ADC->SWTRIG.bit.START = 1;
	while (ADC->STATUS.bit.SYNCBUSY);
}

volatile uint32_t timeoutVal = 0;

void ADC_Handler()
{
	while (ADC->STATUS.bit.SYNCBUSY);
	uint32_t val = ADC->RESULT.reg;

	// Clear the interrupt flag
	ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

	// alternate between channel 0 and 2
	uint32_t channel = ADC->INPUTCTRL.bit.MUXPOS;
	
	switch(channel)
	{
		case 0:
			// FIXME: volume stuff here
			volume = (int32_t)val * 2;
			ADC->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_PIN2_Val;
			while (ADC->STATUS.bit.SYNCBUSY);
			break;
			
		case 2:
			timeoutVal = val;
			// FIXME: set timeout stuff here
			ADC->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_PIN0_Val;
			while (ADC->STATUS.bit.SYNCBUSY);
			break;
			
		default:
			// Probably the first result, toss it.
			ADC->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_PIN0_Val;
			while (ADC->STATUS.bit.SYNCBUSY);
			break;
	}

	ADC->SWTRIG.bit.START = 1;
	while (ADC->STATUS.bit.SYNCBUSY);


}

/*
PA07 - I2S0 /Shutdown 
PA07 - I2S0 DATA  (D9)
PA10 - I2S0 BCLK  (D1)
PA11 - I2S0 LRCLK (D0)
*/


void I2S_Setup()
{
	// Put Generic Clock Generator 3 as source for I2S
	GCLK_CLKCTRL_Type gclk_clkctrl = {
		.bit.WRTLOCK = 0,		/* Generic Clock is not locked from subsequent writes */
		.bit.CLKEN = 1,			/* Enable the Generic Clock */
		.bit.GEN = GENERIC_CLOCK_GENERATOR_MAIN, 	/* Generic Clock Generator 0 (48MHz) is the source */
		.bit.ID = I2S_GCLK_ID_0			/* Generic Clock Multiplexer for TC3  */
	};
	// Write these settings
	GCLK->CLKCTRL.reg = gclk_clkctrl.reg;
	while (GCLK->STATUS.bit.SYNCBUSY);

	// Put Generic Clock Generator 3 as source for I2S
	GCLK_CLKCTRL_Type gclk1_clkctrl = {
		.bit.WRTLOCK = 0,		/* Generic Clock is not locked from subsequent writes */
		.bit.CLKEN = 1,			/* Enable the Generic Clock */
		.bit.GEN = GENERIC_CLOCK_GENERATOR_MAIN, 	/* Generic Clock Generator 0 (48MHz) is the source */
		.bit.ID = I2S_GCLK_ID_1			/* Generic Clock Multiplexer for TC3  */
	};
	// Write these settings
	GCLK->CLKCTRL.reg = gclk1_clkctrl.reg;
	while (GCLK->STATUS.bit.SYNCBUSY);

	PM->APBCMASK.bit.I2S_ = 1;

	// Configure I2S shutdown line
	PORT->Group[PORTA].DIRSET.reg = PORT_PA06;
	PORT->Group[PORTA].OUTCLR.reg = PORT_PA06;	// Turn amplifier off

	// Reconfigure pins for I2S
	PORT->Group[PORTA].PINCFG[7].bit.PMUXEN = 1;  // SD
	PORT->Group[PORTA].PINCFG[10].bit.PMUXEN = 1;  // BCLK
	PORT->Group[PORTA].PINCFG[11].bit.PMUXEN = 1;  // LRCLK

	PORT->Group[PORTA].PMUX[3].bit.PMUXO = 6; // I2S is peripheral function G = 6
	PORT->Group[PORTA].PMUX[5].bit.PMUXE = 6;
	PORT->Group[PORTA].PMUX[5].bit.PMUXO = 6; 

	// f(SCK0) = (GCLK_I2S_0) / (MCKDIV+1)
	// Thus, MCKDIV = 16 for 44.1kHz signal at 32 bits/channel, 2 channels (2822400 Hz bit clock)


	// Full reset
	I2S->CTRLA.bit.ENABLE = 0;
	while(I2S->SYNCBUSY.bit.ENABLE);

	I2S->CTRLA.bit.SWRST = 1;
	while(I2S->SYNCBUSY.bit.SWRST);

	I2S_CLKCTRL_Type clk0;
	clk0.reg = 0;
	clk0.bit.MCKDIV = 16; // See math above
	clk0.bit.MCKEN = 1;
	clk0.bit.BITDELAY = 1; // I2S delay
	clk0.bit.FSWIDTH = 0; // 2 slots
	clk0.bit.NBSLOTS = 1; // 2 slots
	clk0.bit.SLOTSIZE = 3; // 32 bits per slot
	I2S->CLKCTRL[0].reg = clk0.reg;


	I2S_SERCTRL_Type ser0;
	ser0.reg = 0;
	ser0.bit.DATASIZE = I2S_SERCTRL_DATASIZE_32_Val;
	ser0.bit.SERMODE = I2S_SERCTRL_SERMODE_TX_Val;
	ser0.bit.MONO = 1;
	I2S->SERCTRL[0].reg = ser0.reg;


	I2S_CTRLA_Type ctrla;
	ctrla.reg = 0;
	ctrla.bit.CKEN0 = 1;
	ctrla.bit.SEREN0 = 1;
	ctrla.bit.ENABLE = 1;
	I2S->CTRLA.reg = ctrla.reg;
	while(I2S->SYNCBUSY.bit.ENABLE);

	// Attach the interrupt = wont' actually fire until 
	I2S->INTENCLR.bit.TXRDY0 = 1;
	NVIC_SetPriority(I2S_IRQn, 2);	// Set interrupt priority to 1
	NVIC_EnableIRQ(I2S_IRQn);		// Enable I2S Interrupt
}

volatile uint32_t bellOffset = 0;
volatile bool bellOn = false;

void I2S_Handler(void)
{
	static uint8_t lrchan = 0;
	static int32_t audio = 0;

	if (!I2S->INTFLAG.bit.TXRDY0)
		return;

	if (0 == (lrchan % 2))
	{
		bellOffset += 2;
		if (bellOffset >= tinybell_44k_16bit_wav_len)
		{
			bellOffset = 0;
			if (!bellOn)
			{
				PORT->Group[PORTA].OUTCLR.reg = PORT_PA06;	// Disable amplifier
				I2S->INTENCLR.bit.TXRDY0 = 1; // Disable interrupt firing
			}
		}

		uint16_t rawdio = ((uint16_t)tinybell_44k_16bit_wav[bellOffset+1])<<8 | tinybell_44k_16bit_wav[bellOffset];
		audio = (int16_t)rawdio * volume;
	}
	lrchan++;

	I2S->DATA[0].reg = audio;
}

void StartBell()
{
	bellOn = true;
	bellOffset = 0;
	PORT->Group[PORTA].OUTSET.reg = PORT_PA06;	// Enable amplifier
	I2S->INTENSET.bit.TXRDY0 = 1;
}

void StopBell()
{
	bellOn = false;
}


// SCK - PA20
// SDA0 - PA16
// SDA1 - PA17
// SDA2 - PA19
// SDA3 - PA19
#define SDA_PIN_GRP             (PORT_PA16 | PORT_PA17 | PORT_PA18 | PORT_PA19)

static void i2cInit()
{
	PORT->Group[PORTA].DIRCLR.reg = SDA_PIN_GRP; // Make all SDAs inputs for now
	PORT->Group[PORTA].OUTSET.reg = PORT_PA20;	// Made SCK an output and high
	PORT->Group[PORTA].DIRSET.reg = PORT_PA20;

	//Enable pullups on I2C data lines
	PORT->Group[PORTA].PINCFG[PA16].bit.PULLEN = 1;  // SDA0 - PA15
	PORT->Group[PORTA].PINCFG[PA17].bit.PULLEN = 1;  // SDA1 - PA16
	PORT->Group[PORTA].PINCFG[PA18].bit.PULLEN = 1;  // SDA2 - PA20
	PORT->Group[PORTA].PINCFG[PA19].bit.PULLEN = 1;  // SDA3 - PA18
	PORT->Group[PORTA].PINCFG[PA20].bit.PULLEN = 1;  // SCK - PA19

	PORT->Group[PORTA].PINCFG[PA16].bit.DRVSTR = 1;  // SDA0 - PA15
	PORT->Group[PORTA].PINCFG[PA17].bit.DRVSTR = 1;  // SDA1 - PA16
	PORT->Group[PORTA].PINCFG[PA18].bit.DRVSTR = 1;  // SDA2 - PA20
	PORT->Group[PORTA].PINCFG[PA19].bit.DRVSTR = 1;  // SDA3 - PA18
	PORT->Group[PORTA].PINCFG[PA20].bit.DRVSTR = 1;  // SCK - PA19

	PORT->Group[PORTA].PINCFG[PA16].bit.INEN = 1;  // SDA0 - PA15
	PORT->Group[PORTA].PINCFG[PA17].bit.INEN = 1;  // SDA1 - PA16
	PORT->Group[PORTA].PINCFG[PA18].bit.INEN = 1;  // SDA2 - PA20
	PORT->Group[PORTA].PINCFG[PA19].bit.INEN = 1;  // SDA3 - PA18


}

static void sda_low() 
{
	// Set SDA lines low and as outputs
	PORT->Group[PORTA].OUTCLR.reg = SDA_PIN_GRP;
	PORT->Group[PORTA].DIRSET.reg = SDA_PIN_GRP;
	delay_us(10);
}

static void sda_high() 
{
	// Set SDA lines high and as inputs
	PORT->Group[PORTA].DIRCLR.reg = SDA_PIN_GRP;
	PORT->Group[PORTA].OUTSET.reg = SDA_PIN_GRP;
	delay_us(10);
}
static void scl_low() { PORT->Group[PORTA].OUTCLR.reg = PORT_PA20; delay_us(10); }
static void scl_high() { PORT->Group[PORTA].OUTSET.reg = PORT_PA20; delay_us(10); }

void i2cStart()
{
	scl_high();
	sda_low();
	scl_low();
	sda_high();
}

void i2cStop()
{
	scl_low();
	sda_low();
	scl_high();
	sda_high();
}

uint8_t i2cWriteByte(uint8_t byte)
{
	uint8_t i = 0x80, ack = 0x00;
	do
	{
		if(byte & i)
			sda_high();
		else
			sda_low();
		
		scl_high();
		scl_low();
		
		i >>= 1;
	} while(i);

	sda_high();  // Release SDA
	scl_high();

	uint32_t readVal = PORT->Group[PORTA].IN.reg;

	if(!(readVal & PORT_PA16))
		ack |= 0x01;
	if(!(readVal & PORT_PA17))
		ack |= 0x02;
	if(!(readVal & PORT_PA18))
		ack |= 0x04;
	if(!(readVal & PORT_PA19))
		ack |= 0x08;

	scl_low();

	return ack;
}

void i2cReadByte(uint8_t ack, uint8_t* data)
{
	uint8_t i;

	for(i=0; i<4; i++)
		data[i] = 0;

	for(i=0; i<8; i++)
	{
		scl_high();
		data[0] <<= 1;
		data[1] <<= 1;
		data[2] <<= 1;
		data[3] <<= 1;

		uint32_t readVal = PORT->Group[PORTA].IN.reg;
		if(readVal & PORT_PA16)
			data[0] |= 0x01;
		if(readVal & PORT_PA17)
			data[1] |= 0x01;
		if(readVal & PORT_PA18)
			data[2] |= 0x01;
		if(readVal & PORT_PA19)
			data[3] |= 0x01;
		scl_low();
	}
	
	if(ack)
		sda_low();
	scl_high();
	scl_low();
	sda_high();
}

uint8_t writeByte(uint8_t addr, uint8_t cmd, uint8_t writeVal)
{
	uint8_t ack;
	
	i2cStart();
	
	i2cWriteByte(addr << 1);
	i2cWriteByte(cmd);
	ack = i2cWriteByte(writeVal);

	i2cStop();

	return ack;
}

uint8_t readWord(uint8_t addr, uint8_t cmd, uint16_t wdata[])
{
	uint8_t ack = 0x0F, i;
	uint8_t data[4];
	
	i2cStart();
	
	ack &= i2cWriteByte(addr << 1);
	ack &= i2cWriteByte(cmd);

	i2cStart();

	ack &= i2cWriteByte((addr << 1) | 0x01);
	i2cReadByte(1, data);
	for(i=0; i<4; i++)
		wdata[i] = data[i];

	i2cReadByte(0, data);
	for(i=0; i<4; i++)
		wdata[i] |= ((uint16_t)data[i])<<8;
	
	i2cStop();

	return ack;
}

void initializeTMD26711()
{
	// Initialize TMD26711 (bit 0x80 set to indicate command)
	writeByte(TMD26711_ADDR, 0x80|0x00, 0x00);   // Start with everything disabled
	writeByte(TMD26711_ADDR, 0x80|0x01, 0xFF);   // Minimum ATIME
	writeByte(TMD26711_ADDR, 0x80|0x02, 0xFF);   // Maximum integration time
	writeByte(TMD26711_ADDR, 0x80|0x03, 0xFF);   // Minimum wait time
	
	// Note: IRQ not currently used
	writeByte(TMD26711_ADDR, 0x80|0x08, 0x00);   // Set interrupt low threshold to 0x0000
	writeByte(TMD26711_ADDR, 0x80|0x09, 0x00);
	writeByte(TMD26711_ADDR, 0x80|0x0A, 0x00);   // Set interrupt low threshold to 0x0300
	writeByte(TMD26711_ADDR, 0x80|0x0B, 0x03);
	writeByte(TMD26711_ADDR, 0x80|0x0C, 0x10);   // Single out-of-range cycle triggers interrupt

	writeByte(TMD26711_ADDR, 0x80|0x0D, 0x00);   // Long wait disabled
	writeByte(TMD26711_ADDR, 0x80|0x0E, PPULSE_DEFAULT); // Pulse count
	writeByte(TMD26711_ADDR, 0x80|0x0F, 0x28);   // 100% LED drive strength, 4x gain, Use channel 1 diode (ch 1 seems less sensitive to fluorescent) light)

	writeByte(TMD26711_ADDR, 0x80|0x00, 0x27);   // Power ON, Enable proximity, Enable proximity interrupt (not used currently)
}

static uint8_t readTMD26711s()
{
	static uint8_t sensorError[4] = {0,0,0,0};
	static bool detect[4] = {false, false, false, false};
	static uint8_t count[4] = {0, 0, 0, 0};
	uint16_t proximity[4];
	uint8_t ack = 0, i;

	uint8_t retval = 0;

	writeByte(TMD26711_ADDR, 0x80|0x0E, PPULSE_DEFAULT);

	if (sensorError[0] || sensorError[1] || sensorError[2] || sensorError[3])
		initializeTMD26711();

	ack = readWord(TMD26711_ADDR, 0x80|0x20|0x18, proximity);  // Read data register (0x80 = command, 0x20 = auto-increment)

	// Check for missing ACKs, which would indicate a sensor malfunction
	for(i=0; i<4; i++)
	{
		if (0 == (ack & _BV(i)))
		{
			// Sensor's gone wonky, reset it and try again
			if (sensorError[i] < 255)
				sensorError[i]++;

			if (sensorError[i] > SENSOR_ERROR_THRESHOLD)
			{
				detect[i] = false;
				proximity[i] = 0;
				count[i] = 0;
			}

			// This sensor didn't answer, disregard it for now
			continue;
		}

		sensorError[i] = 0;


		if(!detect[i] && (proximity[i] >= PROXIMITY_THRESHOLD))
		{
			// ON debounce
			if(++count[i] > ON_DEBOUNCE_DEFAULT)
			{
				detect[i] = true;
				count[i] = 0;
			}
		}
		else if( (!detect[i] && (proximity[i] < PROXIMITY_THRESHOLD)) 
			|| (detect[i] && (proximity[i] >= PROXIMITY_THRESHOLD)) )
		{
			count[i] = 0;
		}
		else if(detect[i] && (proximity[i] < PROXIMITY_THRESHOLD))
		{
			// OFF debounce
			if(++count[i] > OFF_DEBOUNCE_DEFAULT)
			{
				detect[i] = false;
				count[i] = 0;
			}
		}

		if (detect[i])
			retval |= _BV(i);
	}

	return retval;
}



void initializePWM()
{
	// Put Generic Clock Generator 3 as source for TC3
	GCLK_CLKCTRL_Type gclk_clkctrl = {
		.bit.WRTLOCK = 0,		/* Generic Clock is not locked from subsequent writes */
		.bit.CLKEN = 1,			/* Enable the Generic Clock */
		.bit.GEN = GENERIC_CLOCK_GENERATOR_OSC8M, 	/* Generic Clock Generator 3 is the source */
		.bit.ID = TCC0_GCLK_ID			/* Generic Clock Multiplexer for TC3  */
	};
	// Write these settings
	GCLK->CLKCTRL.reg = gclk_clkctrl.reg;

	PM->APBCMASK.bit.TCC0_ = 1;

	// Disable
	TCC0->CTRLA.bit.ENABLE = 0;
	while(TCC0->SYNCBUSY.bit.ENABLE);

	// Full reset
	TCC0->CTRLA.bit.SWRST = 1;
	while(TCC0->SYNCBUSY.bit.SWRST);

	// Reconfigure some pins for waveform output
	PORT->Group[PORTA].PINCFG[12].bit.PMUXEN = 1;  // MISO - WO6
	PORT->Group[PORTA].PINCFG[14].bit.PMUXEN = 1;  // MOSI - WO4
	PORT->Group[PORTA].PINCFG[15].bit.PMUXEN = 1;  // SCLK - WO5

	PORT->Group[PORTA].PMUX[12/2].bit.PMUXE = 5; // TCC is peripheral function F = 5
	PORT->Group[PORTA].PMUX[14/2].bit.PMUXE = 5;
	PORT->Group[PORTA].PMUX[15/2].bit.PMUXO = 5; 

	TCC0->WAVE.bit.WAVEGEN = TCC_WAVE_WAVEGEN_NPWM_Val;


	TCC0->PERB.reg = 0x8000;
	while(TCC0->SYNCBUSY.bit.PER);
	TCC0->CCB[0].reg = 0;
	while(TCC0->SYNCBUSY.bit.CC0);
	TCC0->CCB[1].reg = 0;
	while(TCC0->SYNCBUSY.bit.CC1);
	TCC0->CCB[2].reg = 0;
	while(TCC0->SYNCBUSY.bit.CC2);
	TCC0->CCB[3].reg = 0;
	while(TCC0->SYNCBUSY.bit.CC3);

	TCC0->CTRLA.bit.ENABLE = 1;
	while(TCC0->SYNCBUSY.bit.ENABLE);

}

void delayInit()
{
	// Put Generic Clock Generator 3 as source for TC3
	GCLK_CLKCTRL_Type gclk_clkctrl = {
		.bit.WRTLOCK = 0,		/* Generic Clock is not locked from subsequent writes */
		.bit.CLKEN = 1,			/* Enable the Generic Clock */
		.bit.GEN = GENERIC_CLOCK_GENERATOR_OSC8M, 	/* Generic Clock Generator 3 is the source */
		.bit.ID = TC3_GCLK_ID			/* Generic Clock Multiplexer for TC3  */
	};
	// Write these settings
	GCLK->CLKCTRL.reg = gclk_clkctrl.reg;

	PM->APBCMASK.bit.TC3_ = 1;

	TC3->COUNT16.CTRLA.bit.ENABLE = 0;
	while(TC3->COUNT16.STATUS.bit.SYNCBUSY);

	TC_CTRLA_Type tc3_ctrla = {
    .bit.SWRST = 0,          // bit:      0  Software Reset                     
    .bit.ENABLE = 0,         // bit:      1  Enable                             
    .bit.MODE = TC_CTRLA_MODE_COUNT16_Val,           // bit:  2.. 3  TC Mode                            
    .bit.WAVEGEN = 0,        // bit:  5.. 6  Waveform Generation Operation      
    .bit.PRESCALER = 3,      // bit:  8x prescaler - 1 count per uS                        
    .bit.RUNSTDBY = 0,       // bit:     11  Run in Standby                     
    .bit.PRESCSYNC = TC_CTRLA_PRESCSYNC_RESYNC_Val      // bit: 12..13  Prescaler and Counter Synchronization
	};

	TC3->COUNT16.CTRLA.reg = tc3_ctrla.reg;

	TC3->COUNT16.COUNT.reg = 1;
	while(TC3->COUNT16.STATUS.bit.SYNCBUSY);

	TC3->COUNT16.CTRLBSET.bit.DIR = 1;
	TC3->COUNT16.CTRLBSET.bit.ONESHOT = 1;

	TC3->COUNT16.CTRLA.bit.ENABLE = 1;
	while(TC3->COUNT16.STATUS.bit.SYNCBUSY);
}

void delay_us(uint16_t microseconds)
{
	if (0 == microseconds)
		return;

	TC3->COUNT16.COUNT.reg = microseconds;
	TC3->COUNT16.CTRLBSET.bit.CMD = 1; // Retrigger
	while(TC3->COUNT16.STATUS.bit.SYNCBUSY);

	while(!TC3->COUNT16.STATUS.bit.STOP);
}

void delay_ms(uint32_t milliseconds)
{
	for(uint32_t i=0; i<milliseconds; i++)
		delay_us(1000);
}

void init()
{
	ClocksInit();
	// Assign the LED0 pin as OUTPUT
	//PORT->Group[LED_RED_PORT].DIRSET.reg = LED_RED_PIN_MASK;
	// Set the LED0 pin level, i.e. put to 3.3V -> LED on
	//PORT->Group[LED_RED_PORT].OUTSET.reg = LED_RED_PIN_MASK;

	// Configure SysTick to trigger an ISR every millisecond using a 48MHz CPU Clock
	SysTick->CTRL = 0;					// Disable SysTick
	SysTick->LOAD = 47999UL;			// Set reload register for 1mS interrupts
	NVIC_SetPriority(SysTick_IRQn, 3);	// Set interrupt priority to least urgency
	SysTick->VAL = 0;					// Reset the SysTick counter value
	SysTick->CTRL = 0x00000007;			// Enable SysTick, Enable SysTick Exceptions, Use CPU Clock
	NVIC_EnableIRQ(SysTick_IRQn);		// Enable SysTick Interrupt
}

void AuxOutputSet()
{
	PORT->Group[PORTA].DIRSET.reg = IO_OUTPUT_AUX_OUT_MASK;
}

void AuxOutputClear()
{
	PORT->Group[PORTA].DIRCLR.reg = IO_OUTPUT_AUX_OUT_MASK;
}

void MainGateDown()
{
	PORT->Group[PORTA].DIRSET.reg = IO_OUTPUT_GATE_A_MASK;
}

void MainGateUp()
{
	PORT->Group[PORTA].DIRCLR.reg = IO_OUTPUT_GATE_A_MASK;
}

void SecondGateDown()
{
	PORT->Group[PORTA].DIRSET.reg = IO_OUTPUT_GATE_B_MASK;
}

void SecondGateUp()
{
	PORT->Group[PORTA].DIRCLR.reg = IO_OUTPUT_GATE_B_MASK;
}


void StartLights()
{
	lightsOn = true;
	AuxOutputSet();
}

void StopLights()
{
	lightsOn = false;
	AuxOutputClear();
}

void initializeIOLines()
{
	// Configure input logic lines
	PORT->Group[PORTB].DIRCLR.reg = (IO_INPUT_AUX_IN_MASK | IO_INPUT_ACTV_MASK | IO_INPUT_WOCC_MASK | IO_INPUT_EOCC_MASK);

	//Enable pullups on logic level inputs
	PORT->Group[PORTB].PINCFG[PB10].bit.PULLEN = 0;
	PORT->Group[PORTB].PINCFG[PB11].bit.PULLEN = 0;
	PORT->Group[PORTB].PINCFG[PB22].bit.PULLEN = 0;
	PORT->Group[PORTB].PINCFG[PB23].bit.PULLEN = 0;
	PORT->Group[PORTB].PINCFG[PB10].bit.INEN = 1;
	PORT->Group[PORTB].PINCFG[PB11].bit.INEN = 1;
	PORT->Group[PORTB].PINCFG[PB22].bit.INEN = 1;
	PORT->Group[PORTB].PINCFG[PB23].bit.INEN = 1;

	// Configure output logic lines, and set all low
	PORT->Group[PORTA].DIRSET.reg = (IO_OUTPUT_AUX_OUT_MASK | IO_OUTPUT_GATE_A_MASK | IO_OUTPUT_GATE_B_MASK);
	PORT->Group[PORTA].OUTCLR.reg = (IO_OUTPUT_AUX_OUT_MASK | IO_OUTPUT_GATE_A_MASK | IO_OUTPUT_GATE_B_MASK);
}

#define INPUT_IR_EAST_APPR   0x00000001
#define INPUT_IR_EAST_ISLD   0x00000002
#define INPUT_IR_WEST_ISLD   0x00000004
#define INPUT_IR_WEST_APPR   0x00000008
#define INPUT_FORCE_ACTIVE   0x00000010
#define INPUT_AUX            0x00000020
#define INPUT_WEST_APPR_OCC  0x00000040
#define INPUT_EAST_APPR_OCC  0x00000080

uint32_t getIOInputs(DebounceState* ioState)
{
	uint32_t retval = 0;
	uint32_t portVal = PORT->Group[PORTB].IN.reg;
	debounce(portVal, ioState);

	if (ioState->debounced_state & IO_INPUT_ACTV_MASK)
		retval |= INPUT_FORCE_ACTIVE;

	if (ioState->debounced_state & IO_INPUT_AUX_IN_MASK)
		retval |= INPUT_AUX;

	if (ioState->debounced_state & IO_INPUT_WOCC_MASK)
		retval |= INPUT_WEST_APPR_OCC;

	if (ioState->debounced_state & IO_INPUT_EOCC_MASK)
		retval |= INPUT_EAST_APPR_OCC;

	return retval;
}

typedef enum
{
	STATE_IDLE                  = 0x00,
	STATE_ACTIVE_UNK            = 0x01,

	STATE_APPROACH_EASTBOUND    = 0x02,
	STATE_APPROACH_EAST_LOCKOUT = 0x03,
	STATE_ACTIVE_EASTBOUND      = 0x04,
	STATE_SHUTDOWN_EASTBOUND    = 0x05,
	STATE_LOCKOUT_EASTBOUND     = 0x06,

	STATE_APPROACH_WESTBOUND    = 0x07,
	STATE_APPROACH_WEST_LOCKOUT = 0x08,
	STATE_ACTIVE_WESTBOUND      = 0x09,
	STATE_SHUTDOWN_WESTBOUND    = 0x0A,
	STATE_LOCKOUT_WESTBOUND     = 0x0B

} CrossingState;

bool isIslandorForceActive(uint32_t detectors)
{
	if (detectors & (INPUT_IR_EAST_ISLD | INPUT_IR_WEST_ISLD | INPUT_FORCE_ACTIVE))
		return true;
	return false;
}

bool isIslandActive(uint32_t detectors)
{
	if (detectors & (INPUT_IR_EAST_ISLD | INPUT_IR_WEST_ISLD))
		return true;
	return false;
}

bool isForceActive(uint32_t detectors)
{
	if (detectors & INPUT_FORCE_ACTIVE)
		return true;
	return false;
}

bool isEastApproachActive(uint32_t detectors)
{
	if (detectors & (INPUT_IR_EAST_APPR | INPUT_EAST_APPR_OCC))
		return true;
	return false;
}

bool isWestApproachActive(uint32_t detectors)
{
	if (detectors & (INPUT_IR_WEST_APPR | INPUT_WEST_APPR_OCC))
		return true;
	return false;
}

int main()
{
	uint32_t sensorState = 0;
	DebounceState ioState;
	CrossingState crossingState = STATE_IDLE;
	bool activateCrossing = false;
	
	volatile uint32_t stateTimer = 0;

	init();
	delayInit();
	initializeIOLines();
	initializePWM();
	i2cInit();
	I2S_Setup();
	ADC_Setup();
	
	initializeTMD26711();
	initDebounceState(&ioState, 0);

	while(1)
	{
		// Get sensors
		if (runFlags & RUN_SENSOR_READ)
		{
			runFlags &= ~(RUN_SENSOR_READ);
			sensorState = readTMD26711s();
			sensorState |= getIOInputs(&ioState);
		}

		// Re-evaluate state
		switch(crossingState)
		{
			
/*#define INPUT_IR_EAST_APPR   0x00000001
#define INPUT_IR_EAST_ISLD   0x00000002
#define INPUT_IR_WEST_ISLD   0x00000004
#define INPUT_IR_WEST_APPR   0x00000008
#define INPUT_FORCE_ACTIVE   0x00000010
#define INPUT_AUX            0x00000020
#define INPUT_WEST_APPR_OCC  0x00000040
#define INPUT_EAST_APPR_OCC  0x00000080*/
			
			case STATE_IDLE:
				//  Lights off, bell off
  				//  If island, go to STATE_ACTIVE_UNK
				//  If east approach, set approach_timer and go to STATE_APPROACH_EASTBOUND
				//  If west approach, set approach_timer and go to STATE_APPROACH_WESTBOUND
				if (isIslandorForceActive(sensorState))
				{
					crossingState = STATE_ACTIVE_UNK;
					stateTimer = ISLAND_TIMER;
				}
				else if (isEastApproachActive(sensorState))
				{
					crossingState = STATE_APPROACH_EASTBOUND;
					stateTimer = EAST_APPROACH_TIMEOUT;
				}
				else if (isWestApproachActive(sensorState))
				{
					crossingState = STATE_APPROACH_WESTBOUND;
					stateTimer = WEST_APPROACH_TIMEOUT;
				}
				else
				{
					activateCrossing = false;
				}
				break;

			case STATE_ACTIVE_UNK:
				// PURPOSE: Turns on the grade crossing if the island sensors suddenly get hit
				// Lights on, bell on
				// If island, go to STATE_ACTIVE_UNK
				// If !island, goto STATE_IDLE
				if (!isIslandorForceActive(sensorState) && 0 == stateTimer)
					crossingState = STATE_IDLE;
				activateCrossing = true;
				break;


			case STATE_APPROACH_EASTBOUND:
				// PURPOSE: Activates crossing when the east approach sensor is triggered
				//  and starts a timer, by which point the train must have hit the island
				// Lights on, bell on
				// If island, go to STATE_ACTIVE_EASTBOUND
				// If approach_timer expired, goto STATE_APPROACH_EAST_LOCKOUT

				if (isForceActive(sensorState))
					crossingState = STATE_ACTIVE_UNK;
				else if (isIslandActive(sensorState))
					crossingState = STATE_ACTIVE_EASTBOUND;

				if (0 == stateTimer)
				{
					crossingState = STATE_APPROACH_EAST_LOCKOUT;
				}
				activateCrossing = true;
				break;  

			case STATE_APPROACH_EAST_LOCKOUT:
				// PURPOSE: Waits for the east approach detector to drop out after we've 
				//    timed out on eastbound approach
				// Lights off, bell off
				// If island, go to STATE_ACTIVE_EASTBOUND
				// If !east approach, goto STATE_IDLE
				// If west approach, goto STATE_APPROACH_WESTBOUND
				
				if (isForceActive(sensorState))
					crossingState = STATE_ACTIVE_UNK;
				else if (isIslandActive(sensorState))
					crossingState = STATE_ACTIVE_EASTBOUND;
				
				if (!isEastApproachActive(sensorState))
					crossingState = STATE_IDLE;

				if (isWestApproachActive(sensorState))
				{
					crossingState = STATE_APPROACH_WESTBOUND;
					stateTimer = WEST_APPROACH_TIMEOUT;
				}
				activateCrossing = false;
				break;

			case STATE_SHUTDOWN_EASTBOUND:
				// Lights on, bell on
				// If island, go to STATE_ACTIVE_EASTBOUND
				// If !island and shutdown_timer expired, set lockout_timer and go to STATE_LOCKOUT_EASTBOUND

				if (isForceActive(sensorState))
					crossingState = STATE_ACTIVE_UNK;
				else if (isIslandActive(sensorState))
					crossingState = STATE_ACTIVE_EASTBOUND;
				else if (0 == stateTimer)
				{
					crossingState = STATE_LOCKOUT_EASTBOUND;
					stateTimer = EAST_LOCKOUT_TIMER;
				}

				activateCrossing = true;
  				break;

			case STATE_LOCKOUT_EASTBOUND:
				// This state prevents retriggering on the west approach sensor for an eastbound
				// Lights off, bell off
				// If island, go to STATE_ACTIVE_EASTBOUND
				// If lockout_timer expired, go to STATE_IDLE
				if (isForceActive(sensorState))
					crossingState = STATE_ACTIVE_UNK;
				else if (isIslandActive(sensorState))
					crossingState = STATE_ACTIVE_EASTBOUND;
				else if (0 == stateTimer && !isWestApproachActive(sensorState))
				{
					crossingState = STATE_IDLE;
				}
				activateCrossing = false;
				break;



			case STATE_APPROACH_WESTBOUND:
				// PURPOSE: Activates crossing when the east approach sensor is triggered
				//  and starts a timer, by which point the train must have hit the island
				// Lights on, bell on
				// If island, go to STATE_ACTIVE_EASTBOUND
				// If approach_timer expired, goto STATE_APPROACH_EAST_LOCKOUT
				if (isForceActive(sensorState))
					crossingState = STATE_ACTIVE_UNK;
				else if (isIslandActive(sensorState))
					crossingState = STATE_ACTIVE_WESTBOUND;

				if (0 == stateTimer)
					crossingState = STATE_APPROACH_WEST_LOCKOUT;

				activateCrossing = true;

				break;  

			case STATE_APPROACH_WEST_LOCKOUT:
				// PURPOSE: Waits for the east approach detector to drop out after we've 
				//    timed out on eastbound approach
				// Lights off, bell off
				// If island, go to STATE_ACTIVE_EASTBOUND
				// If !east approach, goto STATE_IDLE
				// If west approach, goto STATE_APPROACH_WESTBOUND
				if (isForceActive(sensorState))
					crossingState = STATE_ACTIVE_UNK;
				else if (isIslandActive(sensorState))
					crossingState = STATE_ACTIVE_WESTBOUND;
				else if (!isWestApproachActive(sensorState))
					crossingState = STATE_IDLE;
				else if (isEastApproachActive(sensorState))
				{
					crossingState = STATE_APPROACH_EASTBOUND;
					stateTimer = EAST_APPROACH_TIMEOUT;
				}
				activateCrossing = false;

				break;

			case STATE_ACTIVE_WESTBOUND:
				// Lights on, bell on
				// If !island, go to STATE_SHUTDOWN_EASTBOUND
				activateCrossing = true;
				if (isForceActive(sensorState))
					crossingState = STATE_ACTIVE_UNK;
				else if (!isIslandActive(sensorState))
				{
					crossingState = STATE_SHUTDOWN_WESTBOUND;
					stateTimer = WEST_DEPARTURE_TIMEOUT;
				}
  				break;

			case STATE_SHUTDOWN_WESTBOUND:
				// Lights on, bell on
				// If island, go to STATE_ACTIVE_EASTBOUND
				// If !island and shutdown_timer expired, set lockout_timer and go to STATE_LOCKOUT_EASTBOUND
				if (isForceActive(sensorState))
					crossingState = STATE_ACTIVE_UNK;
				else if (isIslandActive(sensorState))
					crossingState = STATE_ACTIVE_WESTBOUND;
				else if (0 == stateTimer)
				{
					crossingState = STATE_LOCKOUT_WESTBOUND;
					stateTimer = WEST_LOCKOUT_TIMER;
				}
				activateCrossing = true;
  				break;

			case STATE_LOCKOUT_WESTBOUND:
				// This state prevents retriggering on the west approach sensor for an eastbound
				// Lights off, bell off
				// If island, go to STATE_ACTIVE_EASTBOUND
				// If lockout_timer expired, go to STATE_IDLE
				if (isForceActive(sensorState))
					crossingState = STATE_ACTIVE_UNK;
				else if (isIslandActive(sensorState))
					crossingState = STATE_ACTIVE_WESTBOUND;
				else if (0 == stateTimer && !(isEastApproachActive(sensorState)))
				{
					crossingState = STATE_IDLE;
				}
				activateCrossing = false;
				break;


			default:
				crossingState = STATE_IDLE;
				break;
		}
		
		if (isIslandorForceActive(sensorState))
			activateCrossing = true;
		else
			activateCrossing = false;
		
		if (activateCrossing && !bellOn)
		{
			StartLights();
			StartBell();
		}
		else if (!activateCrossing && bellOn)
		{
			StopBell();
			StopLights();
		}
	}
}


/*******************************************************************************
 * Function:        void ClocksInit(void)
 *
 * PreCondition:    OSC8M Enabled, CPU and Synch. Bus Clocks at reset condition (1 MHz)
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    CPU and Synchronous Bus Clocks at 48 MHz.
 *					8 MHz Asynchronous Peripheral Clock available on GCLK 3
 *					Flash Read Wait States Adjusted Accordingly.
 *					GCLK_MAIN output on PA28
 *
 * Overview:
 *
 *	Configure a 48 MHz Synchronous Clock for (CPU, AHB, APBx) & Set Flash Wait States for 48MHz
 *		- Use DFLL48M Source (Closed Loop, using external 32.768 kHz crystal)
 *  Configure a 8 MHz Asynchronous Clock on Clock Generator 3 for use with peripherals. 
 *		- Use OSC8M Source
 *
 *	At reset:
 *	- OSC8M clock source is enabled with a divider by 8 (1MHz).
 *	- Generic Clock Generator 0 (GCLKMAIN) is using OSC8M as source.
 *	- OSCULP32K is enabled and fed to Generic Clock Generator 2
 *	- 0 Flash wait states (NVCTRL->CTRLB.RWS = 0)
 *	- Instruction cache is enabled (NVMCTRL->CTRLB.CACHEDIS = 0)
 *	- CPU, APBx clock division is by 1 (CPU, APBx Buses running at 1MHz)
 *	- APBA clocks are connected to EIC, RTC, WDT, GCLK, SYSCTRL,PM, PAC0 peripherals.
 *		
 *	The following steps will be followed to switch over to the 48MHz clock
 *	1) Set Flash wait states for 48 MHz (per Table 37-40 in data sheet)
 *	2) Enable XOSC32K clock (External on-board 32.768Hz oscillator), will be used as DFLL48M reference.
 *	3) Put XOSC32K as source of Generic Clock Generator 1, 
 *	4) Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 0 (DFLL48M reference)
 *	5) Enable DFLL48M clock
 *	6) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.
 *	7) Modify prescaler value of OSC8M to produce 8MHz output
 *	8) Put OSC8M as source for Generic Clock Generator 3
 *	9) Set CPU and APBx BUS Clocks for 48MHz operation
 *	 
 * Notes:
 *
 ******************************************************************************/
void ClocksInit(void)
{
	
	uint32_t tempDFLL48CalibrationCoarse;	/* used to retrieve DFLL48 coarse calibration value from NVM */

	// 1) Set Flash wait states for 48 MHz (per Table 37-40 in data sheet)
	NVMCTRL->CTRLB.bit.RWS = 1;		// 1 wait state required @ 3.3V & 48MHz
	
	// 2) Enable XOSC32K clock (External on-board 32.768kHz oscillator), will be used as DFLL48M reference.
	
	// Configure SYSCTRL->XOSC32K settings
	SYSCTRL_XOSC32K_Type sysctrl_xosc32k = {
		.bit.WRTLOCK = 0,		/* XOSC32K configuration is not locked */
		.bit.STARTUP = 0x2,		/* 3 cycle start-up time */
		.bit.ONDEMAND = 0,		/* Osc. is always running when enabled */
		.bit.RUNSTDBY = 0,		/* Osc. is disabled in standby sleep mode */
		.bit.AAMPEN = 0,		/* Disable automatic amplitude control */
		.bit.EN32K = 1,			/* 32kHz output is disabled */
		.bit.XTALEN = 1			/* Crystal connected to XIN32/XOUT32 */
	};
	// Write these settings
	SYSCTRL->XOSC32K.reg = sysctrl_xosc32k.reg;
	// Enable the Oscillator - Separate step per data sheet recommendation (sec 17.6.3)
	SYSCTRL->XOSC32K.bit.ENABLE = 1;
	
	// Wait for XOSC32K to stabilize
	while(!SYSCTRL->PCLKSR.bit.XOSC32KRDY);
	
	// 3) Put XOSC32K as source of Generic Clock Generator 1

	// Set the Generic Clock Generator 1 output divider to 1
	// Configure GCLK->GENDIV settings
	GCLK_GENDIV_Type gclk1_gendiv = {
		.bit.DIV = 1,								/* Set output division factor = 1 */
		.bit.ID = GENERIC_CLOCK_GENERATOR_XOSC32K	/* Apply division factor to Generator 1 */
	};
	// Write these settings
	GCLK->GENDIV.reg = gclk1_gendiv.reg;
	
	// Configure Generic Clock Generator 1 with XOSC32K as source
	GCLK_GENCTRL_Type gclk1_genctrl = {
		.bit.RUNSTDBY = 0,		/* Generic Clock Generator is stopped in stdby */
		.bit.DIVSEL =  0,		/* Use GENDIV.DIV value to divide the generator */
		.bit.OE = 0,			/* Disable generator output to GCLK_IO[1] */
		.bit.OOV = 0,			/* GCLK_IO[1] output value when generator is off */
		.bit.IDC = 1,			/* Generator duty cycle is 50/50 */
		.bit.GENEN = 1,			/* Enable the generator */
		.bit.SRC = 0x05,		/* Generator source: XOSC32K output */
		.bit.ID = GENERIC_CLOCK_GENERATOR_XOSC32K			/* Generator ID: 1 */
	};
	// Write these settings
	GCLK->GENCTRL.reg = gclk1_genctrl.reg;
	// GENCTRL is Write-Synchronized...so wait for write to complete
	while(GCLK->STATUS.bit.SYNCBUSY);
	

	// 4) Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 0 (DFLL48M reference)

	GCLK_CLKCTRL_Type gclk_clkctrl = {
		.bit.WRTLOCK = 0,		/* Generic Clock is not locked from subsequent writes */
		.bit.CLKEN = 1,			/* Enable the Generic Clock */
		.bit.GEN = GENERIC_CLOCK_GENERATOR_XOSC32K, 	/* Generic Clock Generator 1 is the source */
		.bit.ID = 0x00			/* Generic Clock Multiplexer 0 (DFLL48M Reference) */
	};
	// Write these settings
	GCLK->CLKCTRL.reg = gclk_clkctrl.reg;
	
	// 5) Enable DFLL48M clock
	
	// DFLL Configuration in Closed Loop mode, cf product data sheet chapter
	// 17.6.7.1 - Closed-Loop Operation
	
	// Enable the DFLL48M in open loop mode. Without this step, attempts to go into closed loop mode at 48 MHz will
	// result in Processor Reset (you'll be at the in the Reset_Handler in startup_samd21.c).
	// PCLKSR.DFLLRDY must be one before writing to the DFLL Control register
	// Note that the DFLLRDY bit represents status of register synchronization - NOT clock stability
	// (see Data Sheet 17.6.14 Synchronization for detail)
	while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
	SYSCTRL->DFLLCTRL.reg = (uint16_t)(SYSCTRL_DFLLCTRL_ENABLE);
	while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
	
	// Set up the Multiplier, Coarse and Fine steps
	SYSCTRL_DFLLMUL_Type sysctrl_dfllmul = {
		.bit.CSTEP = 31,		/* Coarse step - use half of the max value (63) */
		.bit.FSTEP = 511,		/* Fine step - use half of the max value (1023) */
		.bit.MUL = 1465			/* Multiplier = MAIN_CLK_FREQ (48MHz) / EXT_32K_CLK_FREQ (32768 Hz) */
	};
	// Write these settings
	SYSCTRL->DFLLMUL.reg = sysctrl_dfllmul.reg;
	// Wait for synchronization
	while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
	
	// To reduce lock time, load factory calibrated values into DFLLVAL (cf. Data Sheet 17.6.7.1)
	// Location of value is defined in Data Sheet Table 10-5. NVM Software Calibration Area Mapping
	
	// Get factory calibrated value for "DFLL48M COARSE CAL" from NVM Software Calibration Area
	tempDFLL48CalibrationCoarse = *(uint32_t*)FUSES_DFLL48M_COARSE_CAL_ADDR;
	tempDFLL48CalibrationCoarse &= FUSES_DFLL48M_COARSE_CAL_Msk;
	tempDFLL48CalibrationCoarse = tempDFLL48CalibrationCoarse>>FUSES_DFLL48M_COARSE_CAL_Pos;
	// Write the coarse calibration value
	SYSCTRL->DFLLVAL.bit.COARSE = tempDFLL48CalibrationCoarse;
	// Switch DFLL48M to Closed Loop mode and enable WAITLOCK
	while(!SYSCTRL->PCLKSR.bit.DFLLRDY);
	SYSCTRL->DFLLCTRL.reg |= (uint16_t) (SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_WAITLOCK);
	
	// 6) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.
	
	// Now that DFLL48M is running, switch CLKGEN0 source to it to run the core at 48 MHz.
	// Enable output of Generic Clock Generator 0 (GCLK_MAIN) to the GCLK_IO[0] GPIO Pin
	GCLK_GENCTRL_Type gclk_genctrl0 = {
		.bit.RUNSTDBY = 0,		/* Generic Clock Generator is stopped in stdby */
		.bit.DIVSEL =  0,		/* Use GENDIV.DIV value to divide the generator */
		.bit.OE = 1,			/* Enable generator output to GCLK_IO[0] */
		.bit.OOV = 0,			/* GCLK_IO[0] output value when generator is off */
		.bit.IDC = 1,			/* Generator duty cycle is 50/50 */
		.bit.GENEN = 1,			/* Enable the generator */
		.bit.SRC = 0x07,		/* Generator source: DFLL48M output */
		.bit.ID = GENERIC_CLOCK_GENERATOR_MAIN			/* Generator ID: 0 */
	};
	GCLK->GENCTRL.reg = gclk_genctrl0.reg;
	// GENCTRL is Write-Synchronized...so wait for write to complete
	while(GCLK->STATUS.bit.SYNCBUSY);
	
	// 7) Modify prescaler value of OSC8M to produce 8MHz output

	SYSCTRL->OSC8M.bit.PRESC = 0;		/* Prescale by 1 */
	SYSCTRL->OSC8M.bit.ONDEMAND = 0 ;	/* Oscillator is always on if enabled */
	
	// 8) Put OSC8M as source for Generic Clock Generator 3
	
	// Set the Generic Clock Generator 3 output divider to 1
	// Configure GCLK->GENDIV settings
	GCLK_GENDIV_Type gclk3_gendiv = {
		.bit.DIV = 1,								/* Set output division factor = 1 */
		.bit.ID = GENERIC_CLOCK_GENERATOR_OSC8M		/* Apply division factor to Generator 3 */
	};
	// Write these settings
	GCLK->GENDIV.reg = gclk3_gendiv.reg;
	
	// Configure Generic Clock Generator 3 with OSC8M as source
	GCLK_GENCTRL_Type gclk3_genctrl = {
		.bit.RUNSTDBY = 0,		/* Generic Clock Generator is stopped in stdby */
		.bit.DIVSEL =  0,		/* Use GENDIV.DIV value to divide the generator */
		.bit.OE = 0,			/* Disable generator output to GCLK_IO[1] */
		.bit.OOV = 0,			/* GCLK_IO[2] output value when generator is off */
		.bit.IDC = 1,			/* Generator duty cycle is 50/50 */
		.bit.GENEN = 1,			/* Enable the generator */
		.bit.SRC = 0x06,		/* Generator source: OSC8M output */
		.bit.ID = GENERIC_CLOCK_GENERATOR_OSC8M			/* Generator ID: 3 */
	};
	// Write these settings
	GCLK->GENCTRL.reg = gclk3_genctrl.reg;
	// GENCTRL is Write-Synchronized...so wait for write to complete
	while(GCLK->STATUS.bit.SYNCBUSY);
	
	// 9) Set CPU and APBx BUS Clocks to 48MHz
	PM->CPUSEL.reg  = PM_CPUSEL_CPUDIV_DIV1 ;
	PM->APBASEL.reg = PM_APBASEL_APBADIV_DIV1_Val ;
	PM->APBBSEL.reg = PM_APBBSEL_APBBDIV_DIV1_Val ;
	PM->APBCSEL.reg = PM_APBCSEL_APBCDIV_DIV1_Val ;
	PM->APBCMASK.bit.PAC2_ = 1;
	PM->APBCMASK.bit.I2S_ = 1;
}

