#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.h"
#include <math.h> //included to support power function

/* Values assignment for Grid Traversal */
#define F 0x11
#define B 0x12
#define R 0x13
#define L 0x14
#define M 0x19
/* Values assignment for Grid Directions */
#define N 0x15
#define S 0x16
#define E 0x17
#define W 0x18

void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
unsigned int row,col;

/* Grid Path */
unsigned int path[]={F,F,F,F,F,F,R,
					R,F,F,F,F,F,L,
					L,F,F,F,F,F,R,
					R,F,F,F,F,F,L,
					L,F,F,F,F,F,R,
					R,F,F,F,F,F,L,
					L,F,F,F,F,F,M};


unsigned int pathindex = 0;
unsigned int dirn = N;
unsigned char sensor[3];
volatile long int ShaftCountLeft = 0; //to keep track of left position encoder 
volatile long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning
unsigned int turnL = 0,turnR =0;

/* Zigbee Initialization */
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x5F; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}
/* Buzzer pin configuration */
void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

/* Buzzer on-off*/
void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

/*Function to configure LCD port*/
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

/* ADC pin configuration */
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

/* Function to configure ports to enable robot's motion */
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}


/* Function to configure INT4 (PORTE 4) pin as input for the left position encoder */
void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; //Enable internal pullup for PORTE 4 pin
}

/* Function to configure INT5 (PORTE 5) pin as input for the right position encoder */
void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x20; //Enable internal pullup for PORTE 4 pin
}


/*Function to Initialize PORTS */
void port_init()
{
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();
	buzzer_pin_config();	
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config	
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
 sei();   // Enables the global interrupt 
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
 EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
 sei();   // Enables the global interrupt 
}

/* ISR for right position encoder*/
ISR(INT5_vect)  
{
 ShaftCountRight++;  //increment right shaft position count
}


/* ISR for left position encoder*/
ISR(INT4_vect)
{
 ShaftCountLeft++;  //increment left shaft position count
}


/* Timer 5 initialized in PWM mode for velocity control
 Prescale:256
 PWM 8bit fast, TOP=0x00FF
 Timer Frequency:225.000Hz */
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

/* Sensor circuit Initialization */
void sensor_pin_config()
{
	DDRB=DDRB | 0x3F;
	PORTB=PORTB &0xFF;
}

/* Polling the piv value continuosly */
int check()
{
	int x=PINB;
	return x;
}

/* ADC initialization */
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

/* Function For ADC Conversion */
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

/* Function To Print Sesor Values At Desired Row And Coloumn Location on LCD*/
void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

/* Function for velocity control */
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

/* Function used for setting motor's direction */
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}

void forward (void) 
{
  motion_set (0x06);//Both wheels forward
}

void stop (void)
{
  motion_set (0x00);//Stop the bot
}

void back (void) //Both wheels backward
{
  motion_set(0x09);
}
void right(void)
{
  motion_set(0x0A);//Right turn
}
void left(void)
{
  motion_set(0x05);//Left Turn
}
void soft_left (void) 
{
 motion_set(0x04); //Left wheel stationary, Right wheel forward
}

void soft_right (void) 
{
 motion_set(0x02); //Left wheel forward, Right wheel is stationary
}

void soft_left_2 (void) 
{
 motion_set(0x01); //Left wheel backward, right wheel stationary
}

void soft_right_2 (void)  
{
 motion_set(0x08); //Left wheel stationary, Right wheel backward
}

/*Function used for turning robot by specified degrees */
void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 ShaftCountRight = 0; 
 ShaftCountLeft = 0; 

 while (1)
 {
  if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
  break;
 }
}

/* Function used for moving robot forward by specified distance */

void linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
 ShaftCountRight = 0;
 while(1)
 {
  if(ShaftCountRight > ReqdShaftCountInt)
  {
  	break;
  }
 } 
 stop(); //Stop action
}

/* Forward the bot by given distance in millimeters */
void forward_mm(unsigned int DistanceInMM)
{
 forward();
 linear_distance_mm(DistanceInMM);
}

/* Backward the bot by given distance in millimeters */
void back_mm(unsigned int DistanceInMM)
{
 back();
 linear_distance_mm(DistanceInMM);
}

/* Degrees by which left turn */
void left_degrees(unsigned int Degrees) 
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 left(); //Turn left
 angle_rotate(Degrees);
}

/* Degrees by which right turn */
void right_degrees(unsigned int Degrees)
{
// 88 pulses for 360 degrees rotation 4.090 degrees per count
 right(); //Turn right
 angle_rotate(Degrees);
}

/* Degrees by which soft left turn */
void soft_left_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_left(); //Turn soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

/* Degrees by which soft right turn */
void soft_right_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_right();  //Turn soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

/* All devices Initialzation */
void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
	uart0_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   //Enables the global interrupts
}

/* Reading values from white-line-sensors */
void read_sensor()
{
	Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
	Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
	Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
}

/* Following grid using given path */
void follow()
{
	if(Center_white_line>0x20 && Left_white_line<0x20 && Right_white_line<0x20) // Center on black line
		{
			velocity(180,180);
		}

	else if((Left_white_line>0x20 && Center_white_line<0x20) ) //left sensor on black line, take right turn
		{
			
			velocity(105,185);
		}

	else if((Right_white_line>0x20 && Center_white_line<0x20)) //right sensor on black line, take left turn
		{
			velocity(185,105);
		}

}

/* Detection of node */
int isPlus()
{

	if((Left_white_line >0x20 && Center_white_line>0x20) || (Right_white_line >0x20 && Center_white_line>0x20))
	{
		buzzer_on();
		_delay_ms(500);
		buzzer_off();
		_delay_ms(500);
		return 1;
	}
	else
	{
		return 0;
	}
}
void turnLeft()
{
	forward_mm(50);
	stop();
	left();
	_delay_ms(200);
	read_sensor();
		 while(Left_white_line <0x40)
		 {
			 read_sensor();
			   left();
		}
	stop();
	_delay_ms(200);
	read_sensor();
	forward();
}

void turnRight()
{
	forward_mm(50);
	stop();
	right();
	_delay_ms(200);
	read_sensor();
		 while(Right_white_line <0x40 )
		 {
			 read_sensor();
			 right();
		 }
	 stop();
	_delay_ms(200);
	read_sensor();
	forward();
}

/* decoding path values to get coordinates */
void orient(int value)
{
	switch(value)
	{

		case F:
		
			if(dirn == N)
			{
				row++;
			}
			else if (dirn == S)
			{
				row--;
			}
			while(isPlus())
			{
				read_sensor();
				follow();
			}

				break;

		case L:
			turnL =1;
			if(dirn == N)
			{
				dirn = W;
			}
			else if (dirn == E)
			{
				dirn = N;
			}
			else if (dirn == W)
			{
				dirn=S;
			}
			else if (dirn == S)
			{
				col++;
				dirn = E;
			}
				break;
		case R:
		
			turnR =1;
			if(dirn == N)
			{
				col++;
				dirn = E;
			}
			else if (dirn == E)
			{
				dirn =S;
			}
			else if (dirn == W)
			{
				dirn = N;
			}
			else if (dirn == S)
			{
				dirn = W;
			}
			
				break;
		case M:
		stop();
		
	}
}

/* Mapping function */
int map(int pindex)
{
	switch(pindex)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 14:
		case 15:
		case 16:
		case 17:
		case 18:
		case 19:
		case 20:
		case 28:
		case 29:
		case 30:
		case 31:
		case 32:
		case 33:
		case 34:
		case 42:
		case 43:
		case 44:
		case 45:
		case 46:
		case 47:
		case 48:return pindex;
		case 7: return 13;
		case 8: return 12;
		case 9: return 11;
		case 10:return 10;
		case 11:return 9;
		case 12:return 8;
		case 13:return 7;
		case 21:return 27;
		case 22:return 26;
		case 23:return 25;
		case 24:return 24;
		case 25:return 23;
		case 26:return 22;
		case 27:return 21;
		case 35:return 41;
		case 36:return 40;
		case 37:return 39;
		case 38:return 38;
		case 39:return 37;
		case 40:return 36;
		case 41:return 35;
	
	}
	return -1;
}

/* Main Function */
int main()
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	sensor_pin_config();
	int value=0;
	unsigned int detected_value;
	
	forward();
	velocity(150,150);
	
	while(1)
	{	
		
		detected_value=check();
			if (detected_value==0x00)//magnet is been detected
			{
				lcd_cursor(1,1);
				lcd_string("Both Side Detect ");
				lcd_cursor(2,1);
				lcd_string(row);
				lcd_cursor(2,5);
				lcd_string(col);
				/* Send coordinate values via Zigbee */	
					UDR0=row;
					UDR0=col;
					UDR0=row;
					UDR0=col+1;
			}
			else if(detected_value==0x80)
			{
				lcd_cursor(1,1);
				lcd_string("Left Hand Found ");	
				lcd_cursor(2,1);
				lcd_string(row);
				lcd_cursor(2,5);
				lcd_string(col);
		
				if(dirn==N)
				{
				/* Send coordinate values via Zigbee */	
					UDR0=row;
					UDR0=col;
				}
				else if(dirn==S)
				{
				/* Send coordinate values via Zigbee */	
					UDR0=row;
					UDR0=col+1;
				}
			}
			else if(detected_value==0x40)
			{
				lcd_cursor(1,1);
				lcd_string("Right Way Exist ");
				lcd_cursor(2,1);
				lcd_string(row);
				lcd_cursor(2,5);
				lcd_string(col);
				
				if(dirn==N)
				{
				/* Send coordinate values via Zigbee */	
					UDR0=row;
					UDR0=col+1;
				}
				else if(dirn==S)
				{
				/* Send coordinate values via Zigbee */	
					UDR0=row;
					UDR0=col;
				}
			}
			
		read_sensor();
		follow();

		if(isPlus()) //check if node is been detected
		{	
			read_sensor();
			value = path[pathindex++];		
			orient(value);
		}
		 if(turnL == 1)
		{
		
		forward_mm(23);
		stop();
		velocity(175,175);
		left_degrees(85);
		read_sensor();
		/* while(Left_white_line <0x40)
		 {
			read_sensor();
			left();
		 }
		 stop();*/
	 	 forward();
		velocity(100,100);
		 turnL = 0;
		}
		 if(turnR == 1)
		{
		forward_mm(22);
		stop();
		velocity(170,170);
		right_degrees(40);
		read_sensor();
		 while(Right_white_line <0x40)
		 {
		 read_sensor();
		 right();
		 }
		stop();
		forward();
		velocity(100,100);
		 turnR = 0;
		}
	
	}
}
