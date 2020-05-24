
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h> // watch dog timer

#define BUZZER PA7
#define BUTTON PB2
#define LED_K PB0 
#define LED_A PB1
#define OneWirePin PA6

const int ClockFactor = 2;
const int LowPulseTime = 10;
union {
    uint16_t fval;
    uint8_t bval[4];
} IntAsByte;

//Pin interrupt 
//  GIMSK  = (1<<PCIE0);  // enable interrupts banks (PortA in this case)
//  PCMSK0 = (1<<PCINT6); // enable PCINT6 mask 0 which correspond to PA6


void set_interrupt() {

	//PORTA = 0xFF; // PULL-UP enabled
	//DDRA = 0x00;  // PortA as input
  //GIMSK  = (1<<PCIE0);  // enable interrupts banks (PortA in this case)
  //PCMSK0 = (1<<PCINT6); // enable PCINT6 mask 0 which correspond to PA6
  //PORTA |= (1 << OneWirePin); // enable pullup 
  
  DDRA &= ~(1 << OneWirePin);    // set pin as input
  PORTA |= (1 << OneWirePin);    // enable pull-up resistor
  
  GIMSK|=(1<<PCIE0);
  PCMSK0|=(1<<PCINT6);  
  //pinMode(OneWirePin, INPUT_PULLUP);   // Set our interrupt pin as input with a pullup to keep it stable

}

void disable_interrupt() {

	GIMSK&=~(1<<PCIE0);
	PCMSK0&=~(1<<PCINT6); //disable interrupt
}


// ---------------- Control of the Mywire pin ----------

void norm_delay_ms(int ms) // this function has a parameter which depends on the clock speed, if this is changed, the parameter should be changed
{
  ms=ms*ClockFactor;
  while (0 < ms)
  {  
    _delay_ms(1);
    --ms;
  }
}

inline void PinLow () {
  DDRA = DDRA | 1<<OneWirePin;  // set as output 
  PORTA &= ~(1 << OneWirePin); // goes low
}

inline void PinRelease () {
	
  DDRA = DDRA & ~(1<<OneWirePin); // set as input (then release the bus)
  PORTA |= (1 << OneWirePin);    // enable pull-up resistor  
  
}

inline uint8_t PinRead () {
  return PINA>>OneWirePin & 1;
}

void LowRelease (const int low, const int high) {
  PinLow();
  norm_delay_ms(low);
  PinRelease();
  norm_delay_ms(high);
}

void OneWireWrite (uint8_t data) {
  int del;
  for (int i = 0; i<8; i++) {
    if ((data & 1) == 1) del = LowPulseTime/4; else del = LowPulseTime;
    LowRelease(LowPulseTime, del);
    data = data >> 1;
  }
}


// ---------------CRC Generation ------------------------


static uint8_t Compute_CRC8_Simple(uint8_t* byte,int numbytes)  // generates 8 bit CRC from a given array of bytes
{
    const uint8_t generator = 0x1D;
    uint8_t crc = 0; /* start with 0 so first byte can be 'xored' in */
	uint8_t currByte =0 ;
    for (int i=0; i<numbytes; i++) 
    {
		currByte=byte[i];
        crc ^= currByte; /* XOR-in the next input byte */

        for (int i = 0; i < 8; i++)
        {
            if ((crc & 0x80) != 0)
            {
                crc = (uint8_t)((crc << 1) ^ generator);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}


//------------ led Control ----------------


void inline ledOn() {
  DDRB |= _BV(LED_A) | _BV(LED_K); //forward bias the LED
  PORTB &= ~_BV(LED_K);            //flash it to discharge the PN junction capacitance
  PORTB |= _BV(LED_A);  
}

void inline ledOff() {
  DDRB &= ~(_BV(LED_A) | _BV(LED_K)); //make pins inputs
  PORTB &= ~(_BV(LED_A) | _BV(LED_K));//disable pullups
}

void ledOnOff() {
    ledOn();
    norm_delay_ms(400);
    ledOff();
    norm_delay_ms(500);
    ledOn();
    norm_delay_ms(400);
    ledOff();
}

//------------ END led Control ----------------

//--------------- sleep / wakeup routines --------------

void inline start_sleep() {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    MCUCR |= _BV(BODS) | _BV(BODSE);    //disable brownout detection during sleep
    MCUCR &=~ _BV(BODSE);
    sleep_cpu();
}



//ISR(ADC_vect) { 
	//nothing, just wake up
//}


// two functions below are needed, this is a workaround
// Function Pototype
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

// Function Implementation
void wdt_init(void)
{
    MCUSR = 0;
    wdt_disable();

    return;
}

ISR(WATCHDOG_vect ) {
   // nothing, just wake up
}

//-----------------------------------------END sleep




//------------------- initialization/setup-------------------

void inline setupGPIO() {
    PORTA |= _BV(PA0);  //nothing
    PORTA &= ~_BV(PA0);                     
    PORTA |= _BV(PA2);  //nothing
    PORTA &= ~_BV(PA2);                     
    PORTA |= _BV(PA3);  //nothing
    PORTA &= ~_BV(PA3);                     
    
    DDRA |= _BV(BUZZER);   //piezo buzzer
    PORTA &= ~_BV(BUZZER);
    

    DDRB |= _BV(PB0);   //nothing
    PORTB &= ~_BV(PB0);
    DDRB |= _BV(PB1);   //nothing
    PORTB &= ~_BV(PB1);
    DDRB |= _BV(PB2);   //sqare wave output
    PORTB &= ~_BV(PB2);
}

void inline setupPowerSaving() {
    DIDR0 |= _BV(ADC1D);   //disable digital input buffer on AIN0 and AIN1
    PRR |= _BV(PRTIM1);                 //disable timer1
    PRR |= _BV(PRTIM0);                 //disable timer0
    ADCSRA &=~ _BV(ADEN);
    PRR |= _BV(PRADC);
    PRR |= _BV(PRUSI);
}


// ------------------ capacitance measurement ------------------

void startExcitationSignal() {
	OCR0A = 0;
	TCCR0A = _BV(COM0A0) |  //Toggle OC0A on Compare Match
			_BV(WGM01);
	TCCR0B = _BV(CS00);
}

void stopExcitationSignal() {
	TCCR0B = 0;
	TCCR0A = 0;
}

uint16_t getADC1() {
    ADCSRA |= _BV(ADPS2); //adc clock speed = sysclk/16
    //ADCSRA |= _BV(ADIE);  // activate interrupt after ADC finishes
    ADMUX |= _BV(MUX0); //select ADC1 as input
    
    ADCSRA |= _BV(ADSC); //start conversion
    
    // sleepWhileADC();
    loop_until_bit_is_clear(ADCSRA, ADSC);  // wait antil the ADC finishes

    uint16_t result = ADCL;
    result |= ADCH << 8;
    
    return 1023 - result;
}

uint16_t getADC1x4average() {

	// read four samples and then average them
	uint16_t sample = getADC1();
	sample += getADC1();
	sample += getADC1();
	sample += getADC1();
	sample = sample >> 2;			// divide by 4 to average the samples

    return sample;
}

uint16_t getCapacitance() {
    //PRR &= ~_BV(PRADC);  //enable ADC in power reduction
    ADCSRA |= _BV(ADEN); // enable ADC  but not start yet
    
    //PRR &= ~_BV(PRTIM0);
    
	startExcitationSignal();

    norm_delay_ms(2);
    getADC1();  // clean the ADC?
    norm_delay_ms(500);  // wait the capacitor stabilizes
    uint16_t result = getADC1x4average();
    norm_delay_ms(2);
    stopExcitationSignal();
    PORTB &= ~_BV(PB2);
    //PRR |= _BV(PRTIM0);
    
    ADCSRA &=~ _BV(ADEN);  // disable ADC
    //PRR |= _BV(PRADC);

    return result;
}




// --------------------------------- THE interrupt from the PIN initiate the answer---------------------------


ISR (PCINT0_vect) {  // this is external interrupt requesting start reading
    
  wdt_enable(WDTO_8S); // watchdog, ATtiny44 should support up to 8 seconds before reset. Every chip has a different timeout lenght
	
  // DIS_OWINT; //disable interrupt
  disable_interrupt();

  // do things here !!
  
  //As this interrupt activates on both edges, we should check that is detect the down-edge that is followed within a certain period by an up edge.
  // read the pin anche chek if it is low 
  if (PinRead () == 0) // detected low edge
  {
	  // start a loop to periodically read the pin value
	  int MaxMs=30;
	  int ms=0;
	  while ((ms < MaxMs)&&(PinRead ()==0))  // wait the signal going up 
	  {  
		norm_delay_ms(1);
		++ms;
		}
	  if ((ms>3) && (ms<(MaxMs-1))) {  
		// good range proceed with sending data to master

		// Get the sensor reading in uint16_t
		

			uint16_t currCapacitance = 0;
				//ledOn();			
			currCapacitance = getCapacitance();
				//ledOff();
		

		IntAsByte.fval = currCapacitance;
		
		// Prepare the array of bytes to be transmitted
		const int numbytestotransmit=2;
		uint8_t DataBytes[numbytestotransmit];
		DataBytes[0]=IntAsByte.bval[0];
		DataBytes[1]=IntAsByte.bval[1];
		
		
		uint8_t CRCbyte=Compute_CRC8_Simple(DataBytes,numbytestotransmit);
		// first down and up is for starting the answer but is not a bit		
		LowRelease (LowPulseTime*1.5, LowPulseTime);
		OneWireWrite (DataBytes[0]);
		OneWireWrite (DataBytes[1]);
		OneWireWrite (CRCbyte);		
		

		//OneWireWrite (0x13);
		
		// end of transmission is a low pulse
		LowRelease (LowPulseTime, 1);

	  }	   
  }
  

  // enable interrupt again
  set_interrupt();
  sei();     // sei and sleep are required
  

  wdt_disable();

}






//-----------------------------------------------------------------

int main (void) {
	
	wdt_disable();  // watch dog timer is enabled by default, it is needed to be disabled otherwise it reset the system in case there is no wdt_reset call in 8 sec (see the loop)

	
	cli(); //disable global interrupts
	
	setupGPIO();

	set_interrupt();

	// the two lines below (clock) influence the ADC value ...
    CLKPR = _BV(CLKPCE);
    CLKPR = _BV(CLKPS1); //clock speed = clk/4 = 2Mhz
  
	norm_delay_ms(200);  // this function has a parameter which depends on the clock speed, if this is changed, the parameter should be changed
	sei();		// turn on interrupts
	ledOnOff();
	

	while(1)
	{

	
	  start_sleep();
	  
	  
      norm_delay_ms(500);
      


	}
	

}
