//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
//#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>

#include "twislave.c"
#include "lcd.c"

#include "adc.c"

//***********************************
//Heizung							*
#define SLAVE_ADRESSE 0x56 //		*
//									*
//***********************************
#define test 0
#define TWI_PORT		PORTC
#define TWI_PIN		PINC
#define TWI_DDR		DDRC

#define SDAPIN		4
#define SCLPIN		5

#define TWI_WAIT_BIT		3
#define TWI_OK_BIT		4
//#define TWI_NEW_BIT		5
#define WDTBIT		7



#define HEIZUNGPORT	PORTD		// Ausgang fuer Heizung
#define UHRBIT       0

#define SERVOPWMBIT  4        // Master soll Daten fuer Servoposition holen

#define RINNEBIT 2

#define SERVOPORT	PORTD		// Ausgang fuer Servo
#define SERVODDR	DDRD		// DDR fuer Servo

#define CONTROL_A 6				// Enable fuer Servo, Active H
#define CONTROL_B 7				// Impuls für Servo

#define FRAME_TIME			20 // Periodendauer Servoimpuls
#define STELLZEITDELAY		10	// Delay fuer Ruekstellung des Servos auf die korrekte Stellung
#define DELTA					5		// Mass fuer Weiterstellen des Servos zur Sicherheit
// Definitionen fuer mySlave
//#define UHREIN 4
//#define UHRAUS 5

// Definitionen Slave Heizung
#define UHREIN			0
#define UHRAUS			1

#define RINNEEIN		2
#define RINNEAUS		3


#define TASTE1		19
#define TASTE2		29
#define TASTE3		44
#define TASTE4		67
#define TASTE5		94
#define TASTE6		122
#define TASTE7		155
#define TASTE8		186
#define TASTE9		205
#define TASTEL		223
#define TASTE0		236
#define TASTER		248
#define TASTATURPORT PORTC
#define TASTATURPIN		3

// Bytes in txbuffer
#define VORLAUF			0	// Byte fuer Vorlauftemperatur
#define RUECKLAUF			1	// Byte fuer Ruecklauftemperatur
#define AUSSEN				2	// Byte fuer Aussentemperatur
#define BRENNER			3	// Byte fuer Brennerstatus
#define RINNE				4	// Byte fuer Dachrinnenstatus

#define ECHO				6	// Byte fuer Echo der Daten vom Master 


#define STATUS				3	// Byte in txbuffer fuer Status
#define BRENNERPIN		2	// PIN 2 von PORT B als Eingang fuer Brennerstatus

#define MANUELL			7	// Bit 7 von Status 
#define MANUELLPIN		5	// Pin 5 von PORT D fuer Anzeige Manuell
#define MANUELLNEU		6	// Pin 6 von Status. Gesetzt wenn neue Schalterposition eingestellt
#define MANUELLTIMEOUT	100 // Loopled-counts bis Manuell zurueckgesetzt wird. 02FF: ca. 100 s
#define LOOPLEDPORT		PORTD
#define LOOPLED			4

#define STARTDELAYBIT	0
#define HICOUNTBIT		1

#define OSZIPORT				PORTD
#define OSZIPORTDDR			DDRD
#define OSZIPORTPIN			PIND
#define PULSA					7

#define OSZIALO OSZIPORT &= ~(1<<PULSA)
#define OSZIAHI OSZIPORT |= (1<<PULSA)
#define OSZIATOG OSZIPORT ^= (1<<PULSA)


void eep_write_wochentag(uint8_t *ablauf[24], uint8_t *tag);
void lcd_puts(const char *s);



void delay_ms(unsigned int ms);


uint8_t EEMEM WDT_ErrCount0;	// Akkumulierte WDT Restart Events
uint8_t EEMEM WDT_ErrCount1;	// WDT Restart Events nach wdt-reset

void delay_ms(unsigned int ms);

static volatile uint8_t					Heizungstatus=0x00;				
static volatile uint8_t					Rinnestatus=0x00;
static volatile uint8_t					Servodatenstatus=0x00;

volatile uint8_t					Echo=0x00;

static volatile uint8_t			Servoimpulsdauer=20;					//	Dauer des Servoimpulses Definitiv
volatile uint8_t					ServoimpulsdauerPuffer=100;		//	Puffer fuer Servoimpulsdauer
static volatile uint8_t			ServoimpulsdauerSpeicher=0;		//	Speicher  fuer Servoimpulsdauer
//volatile uint8_t					Potwert=45;
volatile uint8_t					TWI_Pause=1;
volatile uint8_t					ServoimpulsOK=0;						//	Zaehler fuer richtige Impulsdauer
//uint8_t							Servoposition[]={23,33,42,50,60};
// Richtung invertiert
//volatile uint8_t				Servoposition[]={60,50,42,33,23};
#pragma mark Servowerte
volatile uint8_t					Servoposition[]={100,85,70,60,50};
volatile uint8_t					Stellzeitcounter=3;

volatile uint8_t					Schalterposition=1;// aktuell gueltige Schalterstellung
volatile uint8_t					Masterposition=0;	// Schalterstellung vom Master: Wird waehrend MANUELL gespeichert.

volatile uint16_t					Manuellcounter=0; // Counter fuer Timeout

uint8_t EEMEM WDT_ErrCount;	// Akkumulierte WDT Restart Events




uint8_t Tastenwahl(uint8_t Tastaturwert)
{
//lcd_gotoxy(0,1);
//lcd_putint(Tastaturwert);
if (Tastaturwert < TASTE1)
return 1;
if (Tastaturwert < TASTE2)
return 2;
if (Tastaturwert < TASTE3)
return 3;
if (Tastaturwert < TASTE4)
return 4;
if (Tastaturwert < TASTE5)
return 5;
if (Tastaturwert < TASTE6)
return 6;
if (Tastaturwert < TASTE7)
return 7;
if (Tastaturwert < TASTE8)
return 8;
if (Tastaturwert < TASTE9)
return 9;
if (Tastaturwert < TASTEL)
return 10;
if (Tastaturwert < TASTE0)
return 0;
if (Tastaturwert < TASTER)
return 12;

return -1;
}



void slaveinit(void)
{
 	DDRD |= (1<<UHREIN);			//Pin 0 von PORT D als Ausgang fuer Schalter: ON		
	DDRD |= (1<<UHRAUS);			//Pin 1 von PORT D als Ausgang fuer Schalter: OFF
	DDRD |= (1<<RINNEEIN);		//Pin 2 von PORT D als Ausgang fuer Rinne: ON
 	DDRD |= (1<<RINNEAUS);		//Pin 3 von PORT D als Ausgang fuer Rinne: OFF
	DDRD |= (1<<DDD4);			//Pin 4 von PORT D als Ausgang fuer Loop-LED
	DDRD |= (1<<DDD5);			//Pin 5 von PORT D als Ausgang fuer Manuell
 	//DDRD |= (1<<CONTROL_A);	//Pin 6 von PORT D als Ausgang fuer Servo-Enable
	//DDRD |= (1<<CONTROL_B);	//Pin 7 von PORT D als Ausgang fuer Servo-Impuls
	
	//PORTD |=(1<<PD0);
	//delay_ms(200);
	PORTD &= ~(1<<UHREIN);
	PORTD &= ~(1<<UHRAUS);
	PORTD &= ~(1<<RINNEEIN);
	PORTD &= ~(1<<RINNEAUS);
	//PORTD &= ~(1<<CONTROL_B);
	//PORTD &= ~(1<<CONTROL_A);

	DDRB &= ~(1<<PORTB2);	//Bit 2 von PORT B als Eingang fuer Brennerpin
	PORTB |= (1<<PORTB2);	//HI
	
//	DDRB |= (1<<PORTB2);	//Bit 2 von PORT B als Ausgang fuer PWM
//	PORTB &= ~(1<<PORTB2);	//LO

	DDRB |= (1<<PORTB1);	//Bit 1 von PORT B als Ausgang fuer PWM
	PORTB &= ~(1<<PORTB1);	//LO
	

	//LCD
	DDRB |= (1<<LCD_RSDS_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
 	DDRB |= (1<<LCD_ENABLE_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD
	DDRB |= (1<<LCD_CLOCK_PIN);	//Pin 7 von PORT B als Ausgang fuer LCD

	// TWI vorbereiten
	TWI_DDR &= ~(1<<SDAPIN);//Bit 4 von PORT C als Eingang für SDA
	TWI_PORT |= (1<<SDAPIN); // HI
	
	TWI_DDR &= ~(1<<SCLPIN);//Bit 5 von PORT C als Eingang für SCL
	TWI_PORT |= (1<<SCLPIN); // HI

	SlaveStatus=0;
	SlaveStatus |= (1<<TWI_WAIT_BIT);

	
	DDRC &= ~(1<<PORTC0);	//Pin 0 von PORT C als Eingang fuer Vorlauf 	
//	PORTC |= (1<<DDC0); //Pull-up
	DDRC &= ~(1<<PORTC1);	//Pin 1 von PORT C als Eingang fuer Ruecklauf 	
//	PORTC |= (1<<DDC1); //Pull-up
	DDRC &= ~(1<<PORTC2);	//Pin 2 von PORT C als Eingang fuer Aussen	
//	PORTC |= (1<<DDC2); //Pull-up
	DDRC &= ~(1<<PORTC3);	//Pin 3 von PORT C als Eingang fuer Tastatur 	
	//PORTC &= ~(1<<DDC3); //Pull-up


	
	
}

void initOSZI(void)
{
	OSZIPORTDDR |= (1<<PULSA);
	OSZIPORT |= (1<<PULSA);	
//	OSZIAPORTDDR |= (1<<PULSB);
//	OSZIPORT |= (1<<PULSB);

}



void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

void timer0 (void) 
{ 
// Timer fuer Servo
//	TCCR0 |= (1<<CS00)|(1<<CS02);	//Takt /1024
	TCCR0 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256
	//TCCR0 |= (1<<CS00)|(1<<CS01);	//Takt /64 Intervall 64 us
	
//	TCCR0 |= (1<<CS02);					//Takt 4 MHz /256 Intervall 64 us
	
	TIFR |= (1<<TOV0); 				//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK |= (1<<TOIE0);			//Overflow Interrupt aktivieren
	TCNT0 = 0x00;					//Rücksetzen des Timers
	
//	SERVOPORT |= (1<<CONTROL_B);//	CONTROL_B setzen
}

void setServoA (uint8_t wert)
{
	OCR1A  = 10*wert;    // set Servo 1 to 1ms-position

}


ISR (TIMER0_OVF_vect)
{ 
	Stellzeitcounter++;
	
	if (Stellzeitcounter > (1+abs(Masterposition-Schalterposition))*STELLZEITDELAY)
	{
		//OSZIAHI;
//		SERVOPORT &= ~(1<<CONTROL_B);//	CONTROL_B zuruecksetzen
		Masterposition=Schalterposition;
		Stellzeitcounter=0;
		Servoimpulsdauer=ServoimpulsdauerSpeicher;
		setServoA(Servoimpulsdauer);	 // setzt die Impulsdauer
		ServoimpulsOK=0;
		TCCR0=0; // Timer aussschalten
	}
}


void timer1 (uint16_t wert)
{ 
// Quelle http://www.mikrocontroller.net/topic/103629

//#define FRAME_TIME 20 // msec

ICR1   = FRAME_TIME * 1200;								// PWM cycle time in usec, 50 ms
TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11);	// OC1A/B clr on match, set on TOP
TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS11);		// TOP = ICR1, clk = sysclk/8 (->1us)
TCNT1  = 0;														// reset Timer

OCR1A  = 10*wert;												// set Servo 1 to 1ms-position
OCR1B  = 1000;													// set Servo 1 to 2ms-position  
 
}


ISR(TIMER1_COMPA_vect) // Schaltet Impuls an CONTROL_B ein
{
		
}

ISR (TIMER1_OVF_vect)
{ 
//		TCCR1B=0;
//		TCNT1 = 0x00;
}

void timer2 (uint8_t wert) 
{ 
//	TCCR2 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256

//Takt fuer Servo
	//TCCR2 |= (1<<CS20)|(1<<CS21);	//Takt 1 MHz /64	Intervall 64 us
	TCCR2 |= (1<<CS21)|(1<<CS22);					//Takt 4 MHz /256 Intervall 64 us
	
	TCCR2 |= (1<<WGM21);		//	ClearTimerOnCompareMatch CTC

	//OC2 akt
//	TCCR2 |= (1<<COM20);		//	OC2 Pin zuruecksetzen bei CTC


	TIFR |= (1<<TOV2); 				//Clear TOV2 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK |= (1<<OCIE2);			//CTC Interrupt aktivieren

	TCNT2 = 0x00;					//Zaehler zuruecksetzen
	
	OCR2 = wert;					//Setzen des Compare Registers auf Servoimpulsdauer
} 



ISR(TIMER2_COMP_vect) // Schaltet Impuls an CONTROL_B aus
{
}

void main (void) 
{
	
	
	//PORT2 |=(1<<PC4);
	//PORTC |=(1<<PC5);
	//init_twi_slave (SLAVE_ADRESSE);
	//uint16_t ADC_Wert= readKanal(0);
	//sei();
	wdt_disable();
	MCUSR &= ~(1<<WDRF);
	wdt_reset();
	WDTCR |= (1<<WDCE) | (1<<WDE);
	WDTCR = 0x00;
	slaveinit();
	
	/* initialize the LCD */
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
	delay_ms(100);
	lcd_puts("Guten Tag\0");
	delay_ms(1000);
	lcd_cls();
	lcd_puts("HEIZUNG\0");
	
	HEIZUNGPORT &= ~(1<<UHREIN);//	UHREIN sicher low
	HEIZUNGPORT &= ~(1<<UHRAUS);//	UHRAus sicher low
	
	HEIZUNGPORT &= ~(1<<RINNEEIN);//	RINNEEIN sicher low
	HEIZUNGPORT &= ~(1<<RINNEAUS);//	RINNEAUS sicher low

	uint8_t Tastenwert=0;
	uint8_t TastaturCount=0;
	volatile uint8_t Servorichtung=1;
	
	uint16_t TastenStatus=0;
	uint16_t Tastencount=0;
	uint16_t Tastenprellen=0x01F;
	
	//timer0();
	
	initADC(TASTATURPIN);
	
//	wdt_enable(WDTO_2S);
	
	uint16_t loopcount0=0;
	


	uint8_t twierrcount=0;
	LOOPLEDPORT |=(1<<LOOPLED);
	
	delay_ms(800);
	uint8_t tempWDT_Count=eeprom_read_byte(&WDT_ErrCount);


	//	Zaehler fuer Wartezeit nach dem Start
	uint16_t startdelay0=0x00AF;
	//uint16_t startdelay1=0;

	//Zaehler fuer Zeit von (SDA || SCL = LO)
	uint16_t twi_LO_count0=0;
	uint16_t twi_LO_count1=0;
	//uint8_t SlaveStatus=0x00; //status

	//Zaehler fuer Zeit von (SDA && SCL = HI)
	uint16_t twi_HI_count0=0;

	uint8_t eepromWDT_Count0=eeprom_read_byte(&WDT_ErrCount0); 
	uint8_t eepromWDT_Count1=eeprom_read_byte(&WDT_ErrCount1);
	/*
	eepromWDT_Count0: Zaehler der wdt-Resets mit restart. 
	
	eepromWDT_Count1: Zaehler fuer neuen wdt-Reset. Wenn wdt anspricht, wird der Zaheler erhoeht.
	Beim Restart wird bei anhaltendem LO auf SDA oder SCL gewartet.
	Wenn SCL und SDA beide HI sind, wird der Zaehler auf den Wert von eepromWDT_Count0 gesetzt 
	und der TWI-Slave gestartet.
	
	*/
	
	PORTD &= ~(1<<MANUELLPIN); // MANUELL OFF
	Servoimpulsdauer=Servoposition[Schalterposition];
	timer1(Servoposition[0]);
	
	initOSZI();
	OSZIAHI;
#pragma mark while
	while (1)
	{	
		wdt_reset();
		//Blinkanzeige
		loopcount0++;
		
		if (loopcount0==0x02FF)
		{
			loopcount0=0;
			lcd_gotoxy(0,1);
			//lcd_puts("D \0");
			//lcd_putint2(abs(Masterposition-Schalterposition));
			LOOPLEDPORT ^=(1<<LOOPLED);
			if (Heizungstatus & (1<<MANUELL))
			{
				Manuellcounter++;
				if (Manuellcounter >= MANUELLTIMEOUT)
				{
					ServoimpulsdauerPuffer=0;
					Heizungstatus &= ~(1<<MANUELL); // MANUELL OFF
					Heizungstatus &= ~(1<<MANUELLNEU);
					SERVOPORT &= ~(1<<CONTROL_A);//	CONTROL_A zuruecksetzen: Servo aus
					PORTD &= ~(1<<MANUELLPIN);
					
				}
				
			}
			if (test)
			{
			/*
				uint16_t tempBuffer=0;
				initADC(VORLAUF);
				tempBuffer=readKanal(VORLAUF);
				OSZIAHI;
				
				 lcd_gotoxy(0,1);
				// lcd_puts("     \0");
				 //delay_ms(100);
				 lcd_gotoxy(0,1);
				 lcd_puts("V \0");
				 lcd_putint(tempBuffer>>2);
				 //lcd_put_tempbis99(tempBuffer>>2);
				*/ 

			}
		}

		
		/**	Beginn Startroutinen	***********************/
			
		// wenn Startbedingung vom Master:  TWI_slave initiieren
		if (SlaveStatus & (1<<TWI_WAIT_BIT)) 
		{
			if ((TWI_PIN & (1<<SCLPIN))&&(!(TWI_PIN & (1<<SDAPIN))))// Startbedingung vom Master: SCL HI und SDA LO
			{
			init_twi_slave (SLAVE_ADRESSE);
			sei();
			SlaveStatus &= ~(1<<TWI_WAIT_BIT);
			SlaveStatus |= (1<<TWI_OK_BIT); // TWI ist ON
			
			// StartDelayBit zuruecksetzen
			
			}
		}
		
		else if (test)
		{
		//	init_twi_slave (SLAVE_ADRESSE);
			sei();
		//	SlaveStatus &= ~(1<<TWI_WAIT_BIT);
		//	SlaveStatus |= (1<<TWI_OK_BIT); // TWI ist ON
		
		
		}
							
		
		/**	Ende Startroutinen	***********************/
		
		
		
		
		
		/***** rx_buffer abfragen **************** */
		//rxdata=0;

#pragma mark Servo		
		//***************
		//	Servo
		//***************
		//Heizungstatus=rxbuffer[0];	
		
		// Test
		//rxbuffer[3]=3;
		//lcd_puts("rx3:\0");
		if ((Heizungstatus & (1<< MANUELL))&& (Heizungstatus & (1<<MANUELLNEU))) // Neuer Wert
		{			
				Heizungstatus &= ~(1<<MANUELLNEU);
				if (ServoimpulsdauerSpeicher>Servoimpulsdauer )
				{
					//OSZIALO;
					Servoimpulsdauer=ServoimpulsdauerSpeicher+DELTA; //	Etwas weiter im UZ drehen
					//Servoimpulsdauer=ServoimpulsdauerSpeicher; //	Etwas weiter im UZ drehen
					setServoA(Servoimpulsdauer);
					//Servoimpulsdauer=ServoimpulsdauerSpeicher;
					
					timer0(); // sig_overflow0 stellt das Servo in die korrekte Position
					//OSZIAHI;
					//delay_ms(400); // vorherige Version, stellt das Servo in die korrekte Position
					//Servoimpulsdauer=ServoimpulsdauerSpeicher-2;
					//setServoA(Servoimpulsdauer);
					//delay_ms(400);
					//				Servoimpulsdauer=ServoimpulsdauerSpeicher;
					//setServoA(Servoimpulsdauer);
					/*
					 lcd_gotoxy(0,1);
					 lcd_puts("I\0");
					 lcd_putint2(Servoimpulsdauer);
					 
					 lcd_gotoxy(17,0);
					 lcd_puts("S:\0");
					 lcd_putint1(Schalterposition); // Schalterstellung
					 */
				}
				else if (ServoimpulsdauerSpeicher<Servoimpulsdauer )
				{
					
					Servoimpulsdauer=ServoimpulsdauerSpeicher-DELTA; //	Etwas weiter gegen UZ drehen
					//Servoimpulsdauer=ServoimpulsdauerSpeicher; //	Etwas weiter im UZ drehen
					setServoA(Servoimpulsdauer);
					//Servoimpulsdauer=ServoimpulsdauerSpeicher;
					OSZIALO;
					timer0(); // sig_overflow0 stellt das Servo in die korrekte Position
					//OSZIAHI;
					//delay_ms(400); //
					//Servoimpulsdauer=ServoimpulsdauerSpeicher+2;
					//setServoA(Servoimpulsdauer);
					//delay_ms(400);
					//				Servoimpulsdauer=ServoimpulsdauerSpeicher;
					//setServoA(Servoimpulsdauer);
					/*
					 lcd_gotoxy(0,1);
					 lcd_puts("I\0");
					 lcd_putint2(Servoimpulsdauer);
					 
					 lcd_gotoxy(17,0);
					 lcd_puts("S:\0");
					 lcd_putint1(Schalterposition); // Schalterstellung
					 */
					
				}
		}
		
		if ((SlaveStatus & (1<<TWI_OK_BIT))&& (rxdata) && !(Heizungstatus & (1<< MANUELL)))	//Daten von TWI liegen vor und Manuell ist OFF
		{
			lcd_clr_line(0);
			//lcd_gotoxy(0,0);
			//lcd_puts("rx\0");
			//lcd_putint2(rxbuffer[3]);
			
			{
				if (rxbuffer[3] < 5) // zulaessiger Bereich
				{	
				/*				
					if (Servorichtung && (Schalterposition<4))// vorwärts
					{
						Schalterposition++;
						if (Schalterposition==4)
						{
							Servorichtung=0;
						}
					}
					else if (Schalterposition)
					{
						Schalterposition--;
						if (Schalterposition==0)
						{
							Servorichtung=1;
						}
					}
				*/	
					/*
					lcd_gotoxy(7,0);
					lcd_puts("R:\0");
					lcd_putint2(Servorichtung);
					lcd_puts(" W:\0");
					lcd_putint2(Schalterposition);
					*/
					
					
					Schalterposition=rxbuffer[3];
					
					ServoimpulsdauerPuffer=Servoposition[Schalterposition];
					
				}
				/*
				lcd_gotoxy(0,1);
				lcd_puts("I\0");
				lcd_putint2(Servoimpulsdauer);
				//lcd_gotoxy(4,1);
				lcd_puts(" P\0");
				lcd_putint2(ServoimpulsdauerPuffer);
				
				lcd_puts(" S\0");
				lcd_putint2(ServoimpulsdauerSpeicher);
				//lcd_gotoxy(19,0);
				lcd_puts(" O:\0");
				lcd_putint1(ServoimpulsOK);
				*/
				SERVOPORT &= ~(1<<CONTROL_A);//	CONTROL_A zuruecksetzen: Servo aus
				
				if (!(ServoimpulsdauerPuffer==Servoimpulsdauer))	//	neuer Wert ist anders als aktuelle Impulsdauer
				{
					if (ServoimpulsdauerPuffer==ServoimpulsdauerSpeicher)	// neuer Wert ist schon im Speicher
					{
						ServoimpulsOK++;	//	Zaehler der gleichen Impulse incr
					}
					else
					{
						ServoimpulsdauerSpeicher=ServoimpulsdauerPuffer;
						ServoimpulsOK=0;	//	Zaehler der gleichen Impulse zuruecksetzen
						
					}
					
				}//
				else
				{
					ServoimpulsOK=0;	//Ausreisser
				}
				
				if (ServoimpulsOK>3)	//	neuer Wert ist sicher
				{
					//Masterposition=Schalterposition;
					//SERVOPORT |= (1<<CONTROL_A);//	CONTROL_A setzen: Servo ein
					
					if (ServoimpulsdauerSpeicher>Servoimpulsdauer)
					{
						
						Servoimpulsdauer=ServoimpulsdauerSpeicher+DELTA; //	Etwas weiter im UZ drehen
						setServoA(Servoimpulsdauer);
						OSZIALO;
						timer0(); // sig_overflow0 stellt das Servo in die korrekte Position
						//delay_ms(400); // vorherige Version, stellt das Servo in die korrekte Position
						//Servoimpulsdauer=ServoimpulsdauerSpeicher-2;
						//delay_ms(400);
						
					}
					else
					{
						Servoimpulsdauer=ServoimpulsdauerSpeicher-DELTA; //	Etwas weiter gegen UZ drehen
						setServoA(Servoimpulsdauer);
						OSZIALO;
						timer0(); // sig_overflow0 stellt das Servo in die korrekte Position
						//delay_ms(400); //
						//Servoimpulsdauer=ServoimpulsdauerSpeicher+2;
						//delay_ms(400);
						
					}
					lcd_gotoxy(17,0);
					lcd_puts("S:\0");
					lcd_putint1(Schalterposition); // Schalterstellung
				}//	neuer Wert ist sicher
				
				
				
			}
			
			Heizungstatus=(rxbuffer[0]); // verschoben auf Beginn Servo 
			
			Echo = Heizungstatus; // Kontrolle
			//lcd_gotoxy(17,1);
			//lcd_puts("St:\0");
			//lcd_puthex(Heizungstatus);
			//delay_ms(20);
			
			// TWI_NEW_BIT wird in twislave-ISR gesetzt, wenn alle Daten aufgenommen sind
			
			if (SlaveStatus & (1<<TWI_NEW_BIT))
			{
				SlaveStatus &= ~(1<<TWI_NEW_BIT); // Die Aktionen sollen nur einmal ausgeloest werden
				
				// Brennersteuerung stellen
				if ( Heizungstatus & (1<<UHRBIT))
				{
					//delay_ms(1000);
					//OSZIALO;
					//Schaltuhr ein
					lcd_gotoxy(18,1);
					lcd_puts("B+\0");
					//OSZIAHI;
					HEIZUNGPORT &= ~(1<<UHRAUS);//	UHRAUS sicher low
					HEIZUNGPORT &= ~(1<<UHREIN);//	UHREIN sicher low
					HEIZUNGPORT |= (1<<UHREIN);
					delay_ms(20);
					HEIZUNGPORT &= ~(1<<UHREIN);
				}
				else
				{
					//delay_ms(1000);
					//OSZIALO;
					//Schaltuhr aus
					lcd_gotoxy(18,1);
					lcd_puts("B-\0");
					//OSZIAHI;
					HEIZUNGPORT &= ~(1<<UHREIN);//	UHREIN sicher low
					HEIZUNGPORT &= ~(1<<UHRAUS);//	UHRAUS sicher low
					HEIZUNGPORT |= (1<<UHRAUS);
					delay_ms(20);
					HEIZUNGPORT &= ~(1<<UHRAUS);
				}
				
				
				// 22.01.10	
				Rinnestatus=rxbuffer[2];
				//lcd_gotoxy(17,1);
				//lcd_puthex(Rinnestatus);
				
				// Rinnenheizung stellen
				if ( Rinnestatus  & (1<<0)) // Rinne ein
				{
					//delay_ms(100);
					//OSZIALO;
					//Rinnenheizung ein
					lcd_gotoxy(16,1);
					lcd_puts("R+\0");
					//OSZIAHI;
					HEIZUNGPORT &= ~(1<<RINNEAUS);//	RINNEAUS sicher low
					HEIZUNGPORT &= ~(1<<RINNEEIN);//	RINNEEIN sicher low
					HEIZUNGPORT |= (1<<RINNEEIN); // Impuls auf RINNEEIN
					delay_ms(20);
					HEIZUNGPORT &= ~(1<<RINNEEIN);
				}
				else
				{
					//OSZIALO;
					//delay_ms(100);
					//OSZIALO;
					//Rinnenheizung aus
					lcd_gotoxy(16,1);
					lcd_puts("R-\0");
					//OSZIAHI;
					HEIZUNGPORT &= ~(1<<RINNEEIN);//	RINNEEIN sicher low
					HEIZUNGPORT &= ~(1<<RINNEAUS);//	RINNEAUS sicher low
					HEIZUNGPORT |= (1<<RINNEAUS); // Impuls auf RINNEAUS
					delay_ms(20);
					HEIZUNGPORT &= ~(1<<RINNEAUS);
				}
				
				// end 22.02.10
				




#pragma mark  tx-buffer
				//****************************
				//	tx_buffer laden
				//****************************	
#pragma mark ADC
				
            
            // ***********
            
            // UREF 1.92V
            
            // ***********
				
            // Echo laden
				txbuffer[ECHO] = Echo;
				//lcd_gotoxy(0,0);
				//lcd_puts("     \0");
				//lcd_gotoxy(0,0);
				//lcd_putc('E');
				//lcd_puthex(Echo);
            
            
				//lcd_clr_line(1);
				// Temperatur lesen
				//OSZIALO;
				uint16_t tempBuffer=0;
				initADC(VORLAUF);
				tempBuffer=readKanal(VORLAUF);
				//OSZIAHI;
				if (test)
				{
					
				}
				else
				{
					lcd_gotoxy(0,1);
					lcd_puts("     \0");
					delay_ms(100);
					lcd_gotoxy(0,1);
					lcd_puts("V\0");
					//lcd_putint(tempBuffer>>2);
					lcd_put_tempbis99(tempBuffer>>2);
					
					txbuffer[VORLAUF]=(uint8_t)(tempBuffer>>2);// Vorlauf Byte 0
					//txbuffer[VORLAUF]=(uint8_t)(readKanal(VORLAUF)>>2);// Vorlauf 
					//lcd_gotoxy(0,1);
					//lcd_puts("V\0");
					
					
					initADC(RUECKLAUF);
					
					uint16_t tempBuffer1=readKanal(RUECKLAUF);
					
					lcd_gotoxy(8,1);
					lcd_puts("     \0");
					delay_ms(100);
					lcd_gotoxy(8,1);
					lcd_puts("R\0");
					lcd_put_tempbis99(tempBuffer1>>2);
					
					txbuffer[RUECKLAUF]=(uint8_t)(tempBuffer1>>2);// Ruecklauf Byte 1
					
					
					initADC(AUSSEN);
					tempBuffer=readKanal(AUSSEN);
					txbuffer[AUSSEN]=(uint8_t)(tempBuffer>>2);   // Aussen Byte 2
					lcd_gotoxy(8,0);
					lcd_puts("A\0");
					lcd_put_tempbis99((tempBuffer>>2)-0x20); 
					
					
					//	Brenner abfragen
					txbuffer[STATUS]=(PINB & (1<< BRENNERPIN));	// Brennerstatus Byte 3 
					//lcd_gotoxy(17,1);
					//lcd_puts("B\0");
					
					//	}// if NEW
					
					/*								
					 //	Rinne abfragen
					 if (( Rinnestatus  & (1<<0))
					 {
					 txbuffer[STATUS] |= (1<<RINNE));	// Rinnestatus Byte 2 line 797
					 //lcd_gotoxy(17,1);
					 
					 //lcd_puts("B\0");
					 }
					 else
					 {
					 txbuffer[STATUS]	&= ~(1<<RINNE));
					 }
					 */
					
					
					// 22.8.09 				
					//txbuffer[BRENNER]= ~(PINB & (1<< BRENNERPIN));
					//OSZIALO;
					if (PINB & (1<< BRENNERPIN))
					{
						lcd_puts("-\0");
					}
					else
					{
						lcd_puts("+\0");
					}
					//OSZIAHI;
				}// if test
			}// if NEW	
			
			
			lcd_gotoxy(17,0);
			lcd_puts("S:\0");
			lcd_putint1(Schalterposition); // Schalterstellung
			
			
			rxdata=0;
			//PORTD &= ~(1<<PD3); // 22.01.10 Kollision mit RINNEAUS
			
		}
		

#pragma mark Tastatur 
      
      
		/* ******************** */
		initADC(TASTATURPIN);
		Tastenwert=(readKanal(TASTATURPIN)>>2);
		
//		lcd_gotoxy(3,1);
//		lcd_putint(Tastenwert);
		Tastenwert=0;
		if (Tastenwert>5)
		{
			/*
			 0:											1	2	3
			 1: Uhr ein									4	5	6
			 2:											7	8	9
			 3: Uhr aus									x	0	y
			 4: Schalterpos -
			 5: Manuell ein
			 6: Schalterpos +
			 7: 
			 8: 
			 9: 
			 
			 12: Manuell aus
			 */
			 
			TastaturCount++;
			if (TastaturCount>=200)
			{
				 
				 lcd_gotoxy(17,1);
				 lcd_puts("T:  \0");
				 //lcd_putint(Tastenwert);
				 
				uint8_t Taste=Tastenwahl(Tastenwert);
				//Taste=0;
				 lcd_gotoxy(19,1);
				 lcd_putint1(Taste);
				 //delay_ms(600);
				// lcd_clr_line(1);
				 

				TastaturCount=0;
				Tastenwert=0x00;
				uint8_t i=0;
				uint8_t pos=0;
//				lcd_gotoxy(18,1);
//				lcd_putint2(Taste);
				
				switch (Taste)
				{
					case 0:// Schalter auf Null-Position
					{ 
						if (Heizungstatus & (1<<MANUELL))
						{
							Manuellcounter=0;
							Schalterposition=0;
							Masterposition = rxbuffer[3];
							ServoimpulsdauerSpeicher=Servoposition[0];
							Heizungstatus |= (1<<MANUELLNEU);
							lcd_gotoxy(19,0);
							lcd_putint1(Schalterposition); // Schalterstellung
							/*
							lcd_gotoxy(13,0);
							lcd_puts("S\0");
							lcd_gotoxy(19,0);
							lcd_putint1(Schalterposition); // Schalterstellung
							lcd_gotoxy(0,1);
							lcd_puts("SI:\0");
							lcd_putint(ServoimpulsdauerSpeicher); // Servoimpulsdauer
							lcd_gotoxy(5,0);
							lcd_puts("SP\0");
							lcd_putint(Servoimpulsdauer); // Servoimpulsdauer
							*/
						}
						
					}break;
						
					case 1:	//	Uhr ein
					{ 
					if (Heizungstatus & (1<<MANUELL))
						{
						Manuellcounter=0;
							HEIZUNGPORT &= ~(1<<UHRAUS);//	UHRAUS sicher low
							HEIZUNGPORT &= ~(1<<UHREIN);//	UHREIN sicher low
							HEIZUNGPORT |= (1<<UHREIN);
							delay_ms(20);
							HEIZUNGPORT &= ~(1<<UHREIN);
						}
					}break;
						
					case 2://
					{ 
					
						if (Heizungstatus & (1<<MANUELL))
						{
						Manuellcounter=0;
						uint16_t tempBuffer=0;
						initADC(VORLAUF);
						tempBuffer=readKanal(VORLAUF);
						lcd_gotoxy(0,1);
						lcd_puts("V\0");
						//lcd_putint(tempBuffer>>2);
						lcd_put_tempbis99(tempBuffer>>2);
						}
						
					}break;
						
					case 3: //	Uhr aus
					{ 
						if (Heizungstatus & (1<<MANUELL))
						{
							Manuellcounter=0;
							HEIZUNGPORT &= ~(1<<UHREIN);//	UHREIN sicher low
							HEIZUNGPORT &= ~(1<<UHRAUS);//	UHRAUS sicher low
							HEIZUNGPORT |= (1<<UHRAUS);
							delay_ms(20);
							HEIZUNGPORT &= ~(1<<UHRAUS);
						}
					}break;
						
					case 4://
					{ 
						
					}break;
						
					case 5://
					{ 
						Heizungstatus |= (1<<MANUELL);	// MANUELL ON
						Manuellcounter=0;
						//PORTD |= (1<<PD3);
						//SERVOPORT |= (1<<CONTROL_A);//	CONTROL_A zuruecksetzen: Servo ein
						PORTD |= (1<<MANUELLPIN);
						if (rxbuffer[3]<5)
						{
							Masterposition = rxbuffer[3];
							ServoimpulsdauerSpeicher=Servoposition[rxbuffer[3]];
							Heizungstatus |= (1<<MANUELLNEU);
						}
						/*
						 lcd_gotoxy(13,0);
						 lcd_puts("S\0");
						 lcd_putint1(Schalterposition); // Schalterstellung
						 lcd_gotoxy(0,1);
						 lcd_puts("SP:\0");
						 lcd_putint(ServoimpulsdauerSpeicher); // Servoimpulsdauer
						 lcd_gotoxy(5,0);
						 lcd_puts("SI\0");
						 lcd_putint(Servoimpulsdauer); // Servoimpulsdauer
						 */
					}break;
						
					case 6://
					{ 
					
					}break;
						
					case 7:// Schalter rückwaerts
					{ 
						if ((Heizungstatus & (1<<MANUELL)) &&  Schalterposition)
						{
							Manuellcounter=0;
							Schalterposition--;
							Heizungstatus |= (1<<MANUELLNEU);
							//OSZIALO;
							ServoimpulsdauerSpeicher=Servoposition[Schalterposition];
							lcd_gotoxy(19,0);
							lcd_putint1(Schalterposition); // Schalterstellung
							/*
							lcd_gotoxy(13,0);
							lcd_puts("S\0");
							lcd_putint1(Schalterposition); // Schalterstellung
							lcd_gotoxy(0,1);
							lcd_puts("SP:\0");
							lcd_putint(ServoimpulsdauerSpeicher); // Servoimpulsdauer
							lcd_gotoxy(5,0);
							lcd_puts("SI\0");
							lcd_putint(Servoimpulsdauer); // Servoimpulsdauer
							//OSZIAHI;
							*/
						}
						else 
						{
							
						}
	
					}break;
						
					case 8://
					{ 
						
					}break;
						
					case 9:// Schalter vorwaerts
					{ 
						Manuellcounter=0;
						if ((Heizungstatus & (1<<MANUELL)) && (Schalterposition<4))
						{
							Schalterposition++;
							//OSZIALO;
							ServoimpulsdauerSpeicher=Servoposition[Schalterposition];
							Heizungstatus |= (1<<MANUELLNEU);
							lcd_gotoxy(19,0);
							lcd_putint1(Schalterposition); // Schalterstellung
							/*
							lcd_gotoxy(13,0);
							lcd_puts("S\0");
							lcd_putint1(Schalterposition); // Schalterstellung
							lcd_gotoxy(0,1);
							lcd_puts("SP:\0");
							lcd_putint(ServoimpulsdauerSpeicher); // Servoimpulsdauer
							lcd_gotoxy(5,0);
							lcd_puts("SI\0");
							lcd_putint(Servoimpulsdauer); // Servoimpulsdauer

							//OSZIAHI;
							*/
							
						}
						else 
						{
							//lcd_gotoxy(10,0);
							//lcd_puts("S:!\0");
						}
					

					}break;

					case 10:// *
					{ 
						
					}break;

					case 11://
					{ 
						
					}break;
						
					case 12: // # Normalbetrieb einschalten
					{
						ServoimpulsdauerPuffer=0;
						Heizungstatus &= ~(1<<MANUELL); // MANUELL OFF
						Heizungstatus &= ~(1<<MANUELLNEU);
						SERVOPORT &= ~(1<<CONTROL_A);//	CONTROL_A zuruecksetzen: Servo aus
						PORTD &= ~(1<<MANUELLPIN);
					}
						
				}//switch Tastatur
				
//				delay_ms(400);
//				lcd_gotoxy(18,1);
//				lcd_puts("  ");		// Tastenanzeige loeschen

			}//if TastaturCount	
			
		}//	if Tastenwert
		
		
	}//while


// return 0;
}
