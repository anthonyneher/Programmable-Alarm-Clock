/*
 * Design 1 Final Project
 *
 * Created: 10/9/2019 12:57:02 AM
 * Author : Anthony Neher
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdbool.h>

#define DS3231_addr		0x68


#define bit0 0x01
#define bit1 0x02
#define bit2 0x04
#define bit3 0x08
#define bit4 0x10
#define bit5 0x20
#define bit6 0x40
#define bit7 0x80

typedef struct{
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	bool pm;
	uint8_t wday;
	uint8_t mday;
	uint8_t month;
	uint8_t year;
	
	uint8_t amin;
	uint8_t ahour;
	bool apm;
	bool alarm;
	
	uint8_t stat;
	uint8_t ctrl;
}RTC_info;

char * ALARM = "ALARM    ";

char * pm = "PM";
char * am = "AM";
char * mon = "Monday";
char * tue = "Tuesday";
char * wed = "Wednesday";
char * thr = "Thursday";
char * fri = "Friday";
char * sat = "Saturday";
char * sun = "Sunday";

char * jan = "Jan";
char * feb = "Feb";
char * mar = "Mar";
char * apr = "Apr";
char * may = "May";
char * jun = "Jun";
char * jul = "Jul";
char * aug = "Aug";
char * sep = "Sep";
char * oct = "Oct";
char * nov = "Nov";
char * dec = "Dec";

#define MON 1
#define TUE 2
#define WED 3
#define THU 4
#define FRI 5
#define SAT 6
#define SUN 7


#define f_cpu 8000000
#define Contrast 1
#define Speaker 0
#define RTCaddress 0x00

void init_pins(void);
void init_adc(void);
void init_interrupt(void);
void init_buttons(void);
void init_spi(void);
void init_adc(void);
void init_lcd(void);
void init_i2c(void);
void init_pwm(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(uint8_t data);
uint8_t i2c_read(bool ack);
void RTC_write(uint8_t address, uint8_t data);
uint8_t RTC_read(uint8_t address);
void RTC_update_time(RTC_info * time);
void RTC_read_time(RTC_info * time);
void lcd_cmd(char cmd);
void lcd_char(char chr);
void lcd_string(char * str);
void update_lcd(RTC_info * time, RTC_info * prev, bool alarm);
uint16_t adc_read(void);
void spi_updatedac(uint16_t data, uint8_t device);
uint8_t spi_transmit(uint8_t data);
void print_start(void);
uint8_t dec2reg(uint8_t dec);
uint8_t reg2dec(uint8_t reg);

uint16_t adc_val; 
uint32_t test;
uint8_t i = 0;
volatile uint8_t flag = 0;
bool alarm = false;

uint16_t sine[] =	{0x200,0x27f,0x2f6,0x35e,0x3af,0x3e6,0x3fe,0x3f6,
					0x3ce,0x38a,0x32c,0x2bc,0x240,0x1bf,0x143,0xd3,
					0x75,0x31,0x9,0x1,0x19,0x50,0xa1,0x109,
					0x180,0x200};

uint8_t seconds;
int main(void)
{
	RTC_info time;
	RTC_info prev;
	init_lcd();
	init_pwm();
	init_buttons();
	init_adc();
	init_spi();
	init_interrupt();
	init_i2c();
	spi_updatedac(0x0FF, Contrast);
	//initialize default times
	time.hour = 12;
	time.mday = 1;
	time.min = 0;
	time.wday = 1;
	time.month = 1;
	time.year = 19;
	time.ahour = 12;
	time.apm = 0;
	time.amin = 0;
	time.ctrl = 0;
	RTC_update_time(&time);
	DDRA |= 0x04;//set LED as output (TEST)
	while(1){
		if(((PIND & bit3) == 0) && (PIND & bit4)){
			_delay_us(600);
			while((!(PIND & bit3)) && (PIND & bit4)){//debounce 
				if(!(PIND & bit6)){
					_delay_us(600);
					if(!(PIND & bit6)){//if hour pressed
						time.hour++;
						if(time.hour == 13){
							time.hour = 1;
						}
						if(time.hour == 12){
							time.pm ^= 1;//update am/pm
						}
						RTC_write(0x02,(dec2reg(time.hour) | (time.pm<<5) | 0x40));
						update_lcd(&time, &prev, false);
						prev = time;
						while(!(PIND & bit6));
						_delay_us(600);
					}
				}else if(!(PIND & bit5)){
					_delay_us(600);
					if(!(PIND & bit5)){//if minute pressed
						time.min++;
						if(time.min == 60){
							time.min = 0;
						}
						RTC_write(0x01,dec2reg(time.min));
						update_lcd(&time, &prev, false);
						prev = time;
						while(!(PIND & bit5));
						_delay_us(600);
					}
				}
			}
		}
		if(((PIND & bit4) == 0) && (PIND & bit3)){
			_delay_us(600);
			while((!(PIND & bit4)) && (PIND & bit3)){//debounce 
				update_lcd(&time, &prev, true);
				if(!(PIND & bit6)){
					_delay_us(600);
					if(!(PIND & bit6)){//if hour pressed
						time.ahour++;
						if(time.ahour == 13){
							time.ahour = 1;
						}
						if(time.ahour == 12){
							time.apm ^= 1;//update am/pm
						}
						RTC_write(0x09,(dec2reg(time.ahour) | (time.apm<<5) | 0x40));
						update_lcd(&time, &prev, true);
						prev = time;
						while(!(PIND & bit6));
						_delay_us(600);
					}
				}else if(!(PIND & bit5)){
					_delay_us(600);
					if(!(PIND & bit5)){//if minute pressed
						time.amin++;
						if(time.amin == 60){
							time.amin = 0;
						}
						RTC_write(0x08,dec2reg(time.amin));
						update_lcd(&time, &prev, true);
						prev = time;
						while(!(PIND & bit5));
						_delay_us(600);
					}
				}
			}
		}
		if(!(PINA & 0x08)){
			_delay_us(600);
			if(!(PINA & 0x08)){
				if(time.ctrl & 0x01){
					PORTA &= ~(0x04);
					time.ctrl = 0;
					RTC_write(0x0E,time.ctrl);
					alarm = false;
				}else{
					PORTA |= 0x04;
					time.ctrl = 1;
					RTC_write(0x0E,time.ctrl);
					alarm = true;
				}
				while(!(PINA & 0x08));
				_delay_us(600);
			}
		}
		prev = time;
		RTC_read_time(&time);
		update_lcd(&time, &prev, false);
		if(time.stat & 0x01){
			while ((PINA & 0x08)){
				spi_updatedac((sine[i++]>>1), Speaker);
				if(i==25) i = 0;
			}_delay_us(600);
			time.stat = 0;
			RTC_write(0x0F, time.stat);
		}
		//update lcd brightness based on photoresistor
		adc_val = adc_read();
		test = 100 + (uint8_t)(((float)(adc_val - 300))*0.258);
		if(test>255) test = 255;
		OCR2A = test;
//		_delay_us(100);
	}
/*
    while (1){
//		spi_updatedac((sine[i++]>>1), Speaker);
//		if(i==25) i = 0;
*/
}


//init Functions
void init_interrupt(){
	CLKPR = bit7;
	CLKPR = 0;
	TCCR1B |= bit0;
	TIMSK1 |= bit1;
	OCR1A = (f_cpu / 500) - 1;
	//sei();
}

void init_buttons(void){
	DDRD &= ~(0x78);
	PORTD |= 0x78;
	PORTA |= 0x08;
}

/*
 PD7 = output
 PD7 acts as OC2A
 */
void init_pwm(void){
	DDRD |= bit7;
	PORTD |= bit7;
	OCR2A = 100;
	TCCR2A |= (1<<COM2A1);
	TCCR2A |= (1<<WGM21) | (1<<WGM20);
	TCCR2B |= (1<<CS20);//no pre-scaler
}

/*
 PA0 = ADC
 Sets up ADC pin to measure photo resistor circuit
*/
void init_adc(){
	DDRA &= 0xFC;//ADC Init
	ADMUX = bit6;//AVCC set as reference and ADC0 set as single ended input
	ADCSRA = bit7 | bit2 | bit1 | bit0;//ADC enabled and prescaler is set to 128
}

/*
 PB7 = SCK
 PB5 = MOSI
 PA1 = CS
*/
void init_spi(){
	DDRB |= 0xA0;//set MOSI and SCK as output
	DDRA |= 0x02;//set CS as output
	PORTA |= 0x02;//CS high
	SPCR0 = bit6 | bit4 | bit0;
}


/*
 PB0:3 = D4:D7
 PB4 = E
 PB6 = RS
 */
void init_lcd(void){
	DDRB = 0x1F;//LCD Init
	DDRA |= 0x10;
	lcd_cmd(0x33);
	lcd_cmd(0x32);
	lcd_cmd(0x2C);
	lcd_cmd(0x0C);
	lcd_cmd(0x01);
}

void init_i2c(void){
	TWCR = 0;//reset control register
	TWBR = 46;
//	TWSR = (1<<TWPS1) | (1<<TWPS0);
	TWCR = (1 << TWEN);//enable two wire communication
}

void i2c_start(void){
	/*
	clear the interrupt flag
	set the start condition bit
	keep i2c enabled
	*/
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	//wait for transmission to finish -- might not be necessary
	while(!(TWCR & (1<<TWINT)));
}

void i2c_stop(void){
	/*
	clear the interrupt flag
	set the stop condition bit
	keep i2c enabled
	*/
	TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
	//wait for stop condition to finish
	while(TWCR & (1<<TWSTO));
}

void i2c_write(uint8_t data){
	TWDR = data;
	//keep enabled and clear int bit
	TWCR = (1<<TWEN) | (1<<TWINT);
	while(!(TWCR & (1<<TWINT)));
}


uint8_t i2c_read(bool ack){
	TWCR |= (1 << TWINT) | (ack<<TWEA);
	while(!(TWCR & (1<<TWINT)));
	uint8_t test = TWDR;
	return test;
}

void RTC_write(uint8_t address, uint8_t data){
	i2c_start();
	//7 bit address is shifted left one -- write bit 0
	i2c_write((DS3231_addr<<1) & 0xFE);
	i2c_write(address);
	i2c_write(data);
	i2c_stop();
}

uint8_t RTC_read(uint8_t address){
	i2c_start();
	//7 bit address is shifted left one -- write bit 0
	i2c_write((DS3231_addr<<1) & 0xFE);
	i2c_write(address);
	i2c_stop();
	i2c_start();
	//7 bit address is shifted left one -- read bit 1
	i2c_write((DS3231_addr<<1) | 0x01);
	uint8_t data = i2c_read(false);//bus freezes when ack is sent -- unsure why
	i2c_stop();
	return data;
}

//functions for converting between decimal and register storage for RTC
uint8_t reg2dec(uint8_t reg){
	return ((reg / 16 * 10) + (reg % 16));
}
uint8_t dec2reg(uint8_t dec){
    return ((dec / 10 * 16) + (dec % 10));
}

void RTC_read_time(RTC_info * time){
	time->sec = reg2dec(RTC_read(0x00));
	time->min = reg2dec(RTC_read(0x01));
	time->hour = RTC_read(0x02);
	time->pm = ((time->hour & 0x20)  >> 4);//isolating amx/pm bit
	time->hour = reg2dec(time->hour & 0x1F);
	time->wday = RTC_read(0x03);
	time->mday = reg2dec(RTC_read(0x04));
	time->month = reg2dec(RTC_read(0x05));
	time->year = reg2dec(RTC_read(0x06));
	time->ctrl = RTC_read(0x0E);
	time->stat = RTC_read(0x0F);
	time->ahour = RTC_read(0x09);
	time->apm = ((time->ahour & 0x20)  >> 4);//isolating amx/pm bit
	time->ahour = reg2dec(time->ahour & 0x1F);
	time->amin = reg2dec(RTC_read(0x08));
}

void RTC_update_time(RTC_info * time){
	RTC_write(0x00,dec2reg(time->sec));
	RTC_write(0x01,dec2reg(time->min));
	RTC_write(0x02,(dec2reg(time->hour) | (time->pm<<5) | 0x40));
	RTC_write(0x03,dec2reg(time->wday));
	RTC_write(0x04,dec2reg(time->mday));
	RTC_write(0x05,dec2reg(time->month));
	RTC_write(0x06,dec2reg(time->year));
	RTC_write(0x08,dec2reg(time->amin));
	RTC_write(0x09,(dec2reg(time->ahour) | (time->apm<<5) | 0x40));
	RTC_write(0x0E, time->ctrl);
	RTC_write(0x0F, time->stat);
	RTC_write(0x0A, 0x80);//alarm set on hour and minute match
}

ISR(TIMER1_COMPA_vect){
	//spi_updatedac(wave[i++]);
	if(i == 50) i = 0;
	TCNT1 = 0;
	TIFR1 &= ~bit1;
}



uint8_t spi_transmit(uint8_t data){
	SPDR0 = data;
	while(!(SPSR0 & bit7));
	return SPDR0;
}

void spi_updatedac(uint16_t data, uint8_t device){
	uint16_t message;
	if(device == Contrast){
		message = 0xA000;
	}else if(device == Speaker){
		message = 0x9000;
	}else message = 0;//improper device code
	message |= (data<<2);
	PORTA &= ~(0x02);//CS low
	spi_transmit((uint8_t)(message>>8));
	spi_transmit((uint8_t)(message));
	PORTA |= 0x02;//CS high
}

uint16_t adc_read(void){
	ADCSRA |= bit6;//start conversion
	while(ADCSRA & bit6);
	uint16_t ret = ADC;
	return ret;
}

void lcd_string(char * str){
	volatile int i = 0;
	while(str[i]){
		lcd_char(str[i++]);
	}
}

void lcd_cmd(char cmd){
	char lower = cmd;
	PORTB = 0;
	PORTA &= ~(0x10);
	_delay_ms(5);
	cmd = ((cmd>>4) & 0x0F) | 0x10; //shift command over by one nibble and or with E bit
	PORTB = cmd;
	_delay_ms(5);
	PORTB &= ~(0x10); //set E low to latch data
	
	_delay_ms(5);
	PORTB = ((lower & 0x0F) | 0x10); //send lower nibble and or with E bit
	_delay_ms(5);
	PORTB &= ~(0x10); //set E low to latch data
	_delay_ms(7);
	_delay_ms(5);
}

void lcd_char(char chr){
	char lower = chr;
	PORTA |= 0x10;//RS = 1
	_delay_us(4100);
	chr = ((chr>>4) | 0x10);
	PORTB = chr; //set E and RS high
	_delay_us(4100);
	PORTB &= ~(0x10);//set E low to latch data
	_delay_us(4100);
	lower = (lower | 0x10);
	PORTB = lower; //set E and RS high
	_delay_us(4100);
	PORTB &= ~(0x10);//let E low to latch data
	_delay_us(4100);
}


void update_lcd(RTC_info * time, RTC_info * prev, bool alarm){
	lcd_cmd(0x80);//bring cursor to beginning of second line
	if(alarm){
		time->alarm = true;
	}
	char string[8];
	uint8_t data;
	if(alarm){
		if(time->ahour<10){
			lcd_char('0');
		}
		data = time->ahour;
		ltoa(data, string, 10);
		lcd_string(string);
		lcd_char(':');
	}else{
		if(time->hour<10){
			lcd_char('0');
		}
		data = time->hour;
		ltoa(data, string, 10);
		lcd_string(string);
		lcd_char(':');
	}
	if(alarm){
		data = time->amin;
		if(time->amin<10) lcd_char('0');
		ltoa(data, string, 10);
		lcd_string(string);

		if(time->apm)	lcd_string(pm);
		else			lcd_string(am);
	}else{
			data = time->min;
			if(time->min<10) lcd_char('0');
			ltoa(data, string, 10);
			lcd_string(string);

			if(time->pm)	lcd_string(pm);
			else			lcd_string(am);
	}
	lcd_char(' ');

	if(alarm){
			lcd_string(ALARM);
	}else{
		switch(time->wday){
			case MON:
			lcd_string(mon);
			break;
			case TUE:
			lcd_string(tue);
			break;
			case WED:
			lcd_string(wed);
			break;
			case THU:
			lcd_string(thr);
			break;
			case FRI:
			lcd_string(fri);
			break;
			case SAT:
			lcd_string(sat);
			break;
			case SUN:
			lcd_string(sun);
			break;
		}
	}
	lcd_cmd(0xC0);//move to second line
	switch(time->month){
		case 1:
			lcd_string(jan);
			break;
		case 2:
			lcd_string(feb);
			break;
		case 3:
			lcd_string(mar);
			break;
		case 4:
			lcd_string(apr);
			break;
		case 5:
			lcd_string(may);
			break;
		case 6:
			lcd_string(jun);
			break;
		case 7:
			lcd_string(jul);
			break;
		case 8:
			lcd_string(aug);
			break;
		case 9:
			lcd_string(sep);
			break;
		case 10:
			lcd_string(oct);
			break;
		case 11:
			lcd_string(nov);
			break;
		case 12:
			lcd_string(dec);
			break;
	}
	lcd_char(' ');
	ltoa(time->mday, string, 10);
	if(time->mday<10) lcd_char(' ');
	lcd_string(string);
	lcd_char(',');
	lcd_char(' ');
	ltoa((time->year+2000), string, 10);
	lcd_string(string);
}
