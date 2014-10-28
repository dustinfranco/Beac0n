	//define clock speed
	#define F_CPU 16000000UL
	
	
	//Beac0n: Zero Instruction Guidance Device
	//Used code from: 
	//Peter Fleury (I2C libraries)
	//Jeff Melville and Matt Kuzdeba (GPS NMEA protocol code)
	//Uart by Worgen
	//Also some heavily modified/pulled code from this post:
	//http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=125066&start=0
	
	
	//library includes
    #include <stdio.h>
    #include <stdlib.h>
	#include "math.h"
	#include <avr/io.h>
	#include <avr/interrupt.h>
	#include <avr/pgmspace.h>
	#include <inttypes.h>
	
    //Peter Fleury's I2C lib (slightly modified the assembly file)
	#include "i2cmaster.h"
	//Uart
    #include "uart.c"
    #include "uart.h"
	FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
	
	//NMEA libraries from Jeff Melville and Matt Kuzdeba
	#include "nmea.c"
	#include "nmea.h"


	
	//I2C Compass Variables
	
	long buffer[6];
    long x, y, z = 0;
    short i;
	//set the i2c address
	#define COMPASS 0x3C
	//global heading variable
	float head;
	float PI=3.14159;
	float Heading_Degrees;
	float convRadDeg=180.0/3.14159;
	
	/*****init functions*****/
	
	//compass init
	void comp_init();
	//Calibration Constants - The compass values corresponding to the proper cardinal directions
	int NOR=193;
	int NORE=212;
	int EAS=256;
	int SOUE=270;
	int SOU=334;
	int SOUW=47;
	int WES=110;
	int NORW=170;
	//H is the raw value of the compass reading after heading() is run
	float H;
	//gps init - This is used to check if a valid point has been set or not.  The odds of using
	//the north pole as your first point are very slim!
	float nlat=0.00000;//42.440566;
	float nlon=0.00000;//-76.489864;
	int calcangle();
		int cangle=0;
		int CORangle=0;
		float calc_dist(float clat, float clon, float xlat, float xlon);
	
	//feedback init
	void feedback_init();
	//green pwm value from 0-255
	#define GPWM OCR0B
	//red pwm value from 0-255
	#define RPWM OCR0A
	int but;
	int switcha;
	int switchb;

	/*****Sensor Functions*****/
	
	//Heading  Function Declaration
	float heading();
	
	/*****Feedback Functions*****/
	
	//update LEDs
	void update_LEDs(int angle);
	
	//motor on/off
	void vibrate_motor(int motor_on);

	//play sound
	void play_sound(unsigned int SL, int setf);
	int placehold=0;	
	int sound[255];
	int soundlength;
	//three beeps function
	void threebeeps();
	//check button function
	int check_button();
	//check switches function, returns 0-3
	int check_switches();
	
	//TIMER CONSTANTS
	//time in ms
	//used for scheduling
	volatile unsigned int gpstime=750;
	volatile unsigned int gpscnt=750;
	volatile unsigned int cmptime=200;
	volatile unsigned int cmpcnt=200;
	volatile unsigned int swtime=100;
	volatile unsigned int swcnt=100;
	volatile unsigned int soundcnt;

	//End of declarations
	
	//Interrupts
	ISR (TIMER0_COMPA_vect) {
		//increment system time in ms
		if (gpscnt>0) --gpscnt;
		if (cmpcnt>0) --cmpcnt;
		if (swcnt>0) --swcnt;
		if (soundcnt>0) --soundcnt;
		//if soundcnt is 0, turn off the speaker, this makes the function non blocking
		else OCR1B=0;
	};
	//UART interrupt vector triggers when a character is received
	ISR(USART0_RX_vect)
	{
		//store the character
		char r_char=UDR0;
		//put the character into the nmea buffer
		NmeaPutChar(r_char);
	}
	
	//Main Function
	
	int main()
	{//BEGIN MAIN
		//INITS
		uart_init();
		initGPS();
		stdout = stdin = stderr = &uart_str;
		fprintf(stdout, "uart init complete\n\r");
		i2c_init();
		fprintf(stdout, "i2c init complete\n\r");
		comp_init();
		fprintf(stdout, "compass init complete\n\r");
		feedback_init();
		fprintf(stdout, "feedback init complete\n\r");
		
		UCSR0B|=(1<<RXCIE0);
		//interrupt enable
		sei();
		
		//Set to a point north of Dustin's house
		nlat=42.458775;
		nlon=-76.490234;

		int ledstate=0;			
		//last lattitude and longitude stored
		float llat=0.0000;
		float llon=0.0000;
		//initialize by checking current values
		int button=check_button();
		int mode=check_switches();
		//iterators
		int x=0;
		int z=0;
		int ledstate=1;
		//list of lattitudes and longitudes
		float lonlist[10];
		float latlist[10];
		//populate the list with zeroes
		while(z<10)
		{
			latlist[z]=0.00000;
			lonlist[z]=0.00000;	
			z++;
		}
		fprintf(stdout,"entering while loop\n\r");

		//play sound to let user know the system has initialized
		play_sound(2000,512);
		
/**************************TOP*********************************TOP************
*****************TOP****************************TOP**************************/
/***************************TOP***********************************************
****************************************************TOP**********************/

		while(1)
		{
			//COMPASS FLAGGED
			if(cmpcnt==0)
			{
				//reset compass time
				cmpcnt=cmptime;
				//check that the GPS has a coordinate after initializing
				if(nlat!=nlon)
				{
					/*COMPASS FLAGGED MODE 0
					*Trailblazer Mode*/

					//DO NOTHING IN TRAILBLAZER MODE
					
					/*COMPASS FLAGGED MODE 1
					*Hiker Mode*/
					if(mode==1)
					{
						if(x==0)
							{
								//flash LED quickly red and green to signal
								//the hiker mode is done with it's path
								if(ledstate==1)
								{
								ledstate=0;
								update_LEDs(180);
								}
								else
								{
								ledstate=1;
								update_LEDs(0);
								}
							}
						else
						{
							//if x isn't 0 (the path isn't complete) update the LED
							//with the difference between the heading and the GPS angle
							update_LEDs(abs((int)heading()-CORangle));
						}
					}
					
					/*COMPASS FLAGGED MODE 2 OR MODE 3
					*COMPASS CALIBRATION MODE*/
					if((mode==2)|(mode==3))
					{
						//This enters it's own loop which takes control of the system
						//as long as the mode is 2 or 3
						//three beeps is a blocking function that plays three beeps
						//which signifies that you are in heading calibration mode
						threebeeps();
						//iterator
						int q=0;
						//heading calibration variable
						int headcali;
						while((mode==2)|(mode==3))
						{
							//don't save heading into anything, just run it to update H
							heading();
							//H is the raw compass reading
							headcali=(int)H;
							//Calibrate relative to what it thinks is south
							if(q==0)
							{
								update_LEDs(abs(headcali-SOU));
								//if the button is pressed
								if(!check_button())
								{
									//iterate 
									q++;
									//set south
									SOU=headcali;
									//three beep block before continuing loop
									threebeeps();
								}
							}
							/**********************************************************
							  ALL OF THE NEXT IF STATEMENTS ARE JUST LIKE THE ONE ABOVE
							**********************************************************/
							//southwest
							if(q==1)
							{
								update_LEDs(abs(headcali-SOUW));
								if(!check_button())
								{
									q++;
									SOUW=headcali;
									threebeeps();
								}
							}
							//west
							if(q==2)
							{
							
								update_LEDs(abs(headcali-WES));
								if(!check_button())
								{
									q++;
									WES=headcali;
									threebeeps();
								}
							}
							//northwest
							if(q==3)
							{
								update_LEDs(abs(headcali-NORW));
								if(!check_button())
								{
									q++;
									NORW=headcali;
									threebeeps();
								}
							}
							//north	
							if(q==4)
							{
								update_LEDs(abs(headcali-NOR));
								if(!check_button())
								{
									q++;
									NOR=headcali;
									threebeeps();
								}
							}
							//northeast
							if(q==5)
							{
								update_LEDs(abs(headcali-NORE));
								if(!check_button())
								{
									q++;
									NORE=headcali;
									threebeeps();
								}
							}
							//east
							if(q==6)
							{
								update_LEDs(abs(headcali-EAS));
								if(!check_button())
								{
									q++;
									EAS=headcali;
									threebeeps();
								}
							}
							//southeast
							if(q==7)
							{
								update_LEDs(abs(headcali-SOUE));
								if(!check_button())
								{
									q++;
									SOUE=headcali;
									threebeeps();
								}
							}
							//THIS one is when you're done calibrating, now it waits to leave the mode
							if(q==8)
							{
								GPWM=0;
								RPWM=255;
							}
							//Read the mode after all the if statements to see if you should leave the loop
							//all changes will be saved
							mode=check_switches();
						}
						//play sound to let user know they left the loop
						play_sound(500,512);
					}
				}
			}
			
			//GPS FLAGGED
			if(gpscnt==0)
			{
				//reset the gps time
				gpscnt=gpstime;
				//process Uart, saving the last lattitude and longitude
				llat=lat;
				llon=lon;
				//nmea process task updates lat and lon, given in the nmea.c lib
				NmeaProcessTask();
				
				//if the gps hasn't picked up signal yet blink the LED
				if(lon==lat)
				{
					if(ledstate==0)
					{
						update_LEDs(0);
						ledstate=2;
					}
					if(ledstate==1)
					{
						update_LEDs(180);
						ledstate=0;
					}
					if(ledstate==2)
					ledstate=1;
				}
				//BIG ELSE CASE INCOMING, this is what to do if there is valid lat and lon
				else
				{
				/* GPS FLAGGED MODE 0
				*  Trailblazer Mode
				*/
					if(mode==0)
					{
						//stable input, allow updates
						//stable input: lat/lon are the same as last update
						//latitude isn't longitude (update sometimes is repeatedly 0.000)
						if((lat==llat)&&(lon==llon)&&(lat!=lon))
						{	
						//check to see that we are far enough away from the lsat coordinate set
						//skip this if x is 0
							if(x>0)
							{
								//calculate the distance to the lat coordinate
								float discal=calc_dist(latlist[x-1],lonlist[x-1],lat,lon);
								//
								if(discal>0.0003)
								{				
									//you are at a safe distance and input is stable
									GPWM=0;
									RPWM=255;
								}
								else
								{
									//you are too close to set anohter point
									GPWM=255;
									RPWM=0;
								}
							}
								
							if(x==0)
							{
								//you can set a point wherever you want as long
								//as the input is stable
								GPWM=0;
								RPWM=255;
							}	
						}
						else
						{
							//unstable input, red LED
							GPWM=255;
							RPWM=0;
						}
					}
					/*
					*
					*GPS FLAGGED MODE 1
					*Hiker Mode
					*/

					if(mode==1)
					{
						//distance variable
						float dcal;
						//if you are not at the end of hte path (x=0)
						if(x>0)
						{
						dcal=calc_dist(latlist[x-1],lonlist[x-1],lat,lon);
						if(dcal>0.0003)
							{
							//set blue light intensity, this is not an equation,
							//just arbitrary unitless distances that can be modified
								if(dcal>=.0010)
									OCR1A=1020;
								if(dcal<.0010)
									OCR1A=800;
								if(dcal<.0007)
									OCR1A=700;
								if(dcal<.0005)
									OCR1A=400;
								if(dcal<.0004)
									OCR1A=0;
							}
						else
						{
						//this shouldn't ever happen, but if it does then set it off
						OCR1A=1023;
						}
						//if you have reached the next waypoint (within a certain distance of it)
						if(dcal<0.000300)
						{
							//play sound and decrement x
							play_sound(1000,512);
							x--;
							//paranoid case, should never happen
							if(x<0)
							{
								x=0;
							}
							if(x>0)
							{
							//as long as x is greater than 0, set the next coordinates as
							//the target location.
								nlat=latlist[x-1];
								nlon=lonlist[x-1];
								//fprintf(stdout,"nloc: %f    %f\n\r", nlat,nlon);
							}
						}
						}
						//update the angle of the GPS relative to it's next location NO MATTER WHAT
						calcangle();
	
				}
				}
			}
			//END of GPS call

			//////////////
			//SWITCH CHECK
			///////////////
			if(swcnt==0)
			{	
				//reset switch time
				swcnt=swtime;
				//set mode by checking switches
				mode=check_switches();
				//if the mode is 0 and the green LED is on (which means stable input
				if((!check_button())&&(mode==0)&&(GPWM==0))
				{
					//set up the coordinate differently if x==0
					//BUTTON FOR TRAILBLAZER ONLY
					//set up if statement so that it is readable
					if(
					(x!=0)
					&&
					(x<10)
					&&
					(calc_dist(latlist[x-1],lonlist[x-1],lat,lon)>.0003)
					)
					{
						//add your current coordinate to the list
						latlist[x]=lat;
						lonlist[x]=lon;
						//uart send, if you're using a computer, this data is sent
						//only on a point record
						fprintf(stdout, "recP %f   %f\n\r", latlist[x],lonlist[x]);
						x++;
					}	
					//if x == 0 then you need not (and cant!) calculate distance
					//so skip that
					if(x==0)
					{
						//same as the if statment above
						latlist[x]=lat;
						lonlist[x]=lon;
						fprintf(stdout, "recP %f   %f\n\r", latlist[x],lonlist[x]);
						x++;
					}
				}

				//if the mode is 2, then do nothing
			}
		}
		
	}//END MAIN
	
/******************BOTTOM*****************************************************
************************************************BOTTOM******BOTTOM***********/	
/************************BOTTOM***********************************************
*****************************************************************************/	

	/***********************/
	//DEVICE INIT FUNCTIONS//
	/***********************/
	//Done: comp_init()
	//To Do: accel_init(),gps_init(),feedback_init(),accel_init()
	
	/////////////////////////
	//COMPASS INIT FUNCTION//
	/////////////////////////
	
	void comp_init()
	{
		//00=configuration register A
	  	i2c_start_wait(COMPASS + I2C_WRITE);
        i2c_write(0x00);
        i2c_write(0x70);//turn on 8 sample averaging
        i2c_stop();
		//01=configuration register B
        i2c_start_wait(COMPASS + I2C_WRITE);
        i2c_write(0x01);
        i2c_write(0xA0);       
        i2c_stop();
		//02 is mode register
        i2c_start_wait(COMPASS + I2C_WRITE);
        i2c_write(0x02);
        i2c_write(0x00);
        i2c_stop();
	}
	
	//////////////////////////
	//FEEDBACK INIT FUNCTION//
	//////////////////////////
	
	void feedback_init()
	{
		//LED INIT
		//Turn on PWM for OCR0A and OCR0B
		TCCR0A = /*(1<<COM0A0) |*/ (1<<COM0A1) | (1<<WGM00) | (1<<WGM01) | /*(1<<COM0B0) |*/ (1<<COM0B1);
		//Set Port Directions
		DDRB = (1<<PINB3|1<<PINB4) ;
		//Set the PWM
		TCCR0B = 3 ;
		TIMSK0 = (1<<OCIE0A); //turn on timer 0 compare match ISR
		RPWM=0;
		GPWM=0;	
		//Switches and Button Init
		//Or them to not interfere with I2C SDA and SCL on pins C.0 and C.1
		DDRC=0b00000000 |DDRC;
		PORTC=0b11111100 |PORTC;
		//Speaker Init - Revisit later
		DDRD = (1<<PIND4 ) | (1<<PIND5);
		TCCR1A = (1<<COM1A1)|(1<<WGM10) | (1<<WGM11) |  (1<<COM1B1) ;
		TCCR1B = 2;
		OCR1B=512;
		OCR1A=1020;
	}
	


	////////////////////////
	//Get Heading Function//
	////////////////////////
	
	//Description: Gets your current heading 

	float heading()
	{                    
		//raw value variables
	    int16_t raw_x = 0; 
	    int16_t raw_y = 0; 
	    int16_t raw_z = 0; 

	    i2c_start_wait(COMPASS + I2C_WRITE); 
	    i2c_write(0x03); //set pointer to X-axis MSB 
	    i2c_stop(); 
		//repeatedly ask for i2c read
	    i2c_rep_start(COMPASS+I2C_READ); 

	    raw_x = ((uint16_t)i2c_readAck())<<8; 
	    raw_x |= i2c_readAck(); 

	    raw_z = ((uint16_t)i2c_readAck())<<8; 
	    raw_z |= i2c_readAck(); 

	    raw_y = ((uint16_t)i2c_readAck())<<8; 
	    raw_y |= i2c_readNak(); 

	    i2c_stop(); 
		//RAW HEADING 
	    H = atan2(raw_y,raw_x)* 57 + 180; 

		int HCOR=0;
		//Correct the angle
		//SW-W 0-45
		if((SOUW<H)&&(H<=WES))
		{
			HCOR=(int)((H-SOUW)*(45.0/(WES-SOUW)));
		}
		//W-NW 45-90
		if((WES<H)&&(H<=NORW))
		{
			HCOR=(int)(45.0+((H-WES)*(45.0/(NORW-WES))));
		}
		//NW-N 90-135
		if((NORW<H)&&(H<=NOR))
		{
			HCOR=(int)(90.0+((H-NORW)*(45.0/(NOR-NORW))));
		}
		//N-NE 135-180
		if((NOR<H)&&(H<=NORE))
		{
			HCOR=(int)(135.0+((H-NOR)*(45.0/(NORE-NOR))));
		}
		//NE-E 180-225
		if((NORE<H)&&(H<=EAS))
		{
			HCOR=(int)(180.0+((H-NORE)*(45.0/(EAS-NORE))));
		}
		//E-SE 225-270
		if((EAS<H)&&(H<=SOUE))
		{
			HCOR=(int)(225.0+((H-EAS)*(45.0/(SOUE-EAS))));
		}
		//SE-S 270-315
		if((SOUE<H)&&(H<=SOU))
		{
			HCOR=(int)(270.0+((H-SOUE)*(45.0/(SOU-SOUE))));
		}
		//S-SW 315-360
		if((SOU<H)&&(H<=360))
		{
			HCOR=(int)(315.0+((H-SOU)*(45.0/(360-SOU+SOUW))));
		}
		if(H<=SOUW)
		{
		   HCOR=(int)(315.0+((H+360-SOU)*(45.0/(360-SOU+SOUW))));
		}
	   return (float)HCOR;
	}
	
	///////////////
	//calc_dist()//
	///////////////
	
	//Description: Calculates Euclidian distance from user to next waypoint
	//returns your distance from the next waypoint might be easier wiht pointers
	
	float calc_dist(float clat, float clon, float xlat, float xlon)
	{
		float dist;
		dist=sqrt(pow((clat-xlat),2)+pow((clon-xlon),2));
		return dist;
	}
	
	
	////////////////
	//calc_angle()//
	////////////////
	
	//Description: Calculates angle difference between current and next waypoint
	//Returns an int between 0-180 pointers might be necesssary
	int calcangle()
	{			
		cangle=(int)(atan2((nlat-lat),(nlon-lon))*(180/PI));
		if((cangle<=-135)&&(cangle>=-180))
		{
			CORangle=495+cangle;
		}
		else
		{
			CORangle=cangle+135;
		}
		CORangle=360-CORangle;
		return CORangle;
	}	
	
	/********************/
	//Feedback Functions//
	/********************/
	//list of functions here:
	//update_LEDs()
	//check_button()
	//threebeeps()
	//vibrate_motor()
	//play_sound()
	//check_switches()
	// play_sound()
	//vibrate_motor(True/False)
	//update_leds()
	
	////////////////
	//play_sound()//
	////////////////
	
	//Description: Plays sound, taking sound legnth and OCR1B value as arguments
	void play_sound(unsigned int SL, int setf)
	{
		soundcnt=SL;
		OCR1B=setf;
	}
	
	/////////////////////////////
	//vibrate_motor(True/False)//
	/////////////////////////////
	
	//Description: Turn the motor on/off
	//motor is on C.5
	//not in use, but still might be useful in the future
	void vibrate_motor(int motor_on)
	{
		if(motor_on==1)
		{
			DDRC=0b00100000 |DDRC;
		}
		else 
		{
			DDRC=0b00000000 |DDRC;
		}
	}
			
	
    ////////////////////
	//check_switches()//
	////////////////////
	
	//Description: checks switches and returns their values
	//debouncing should be done in hardware
	//switches are on C.3 and C.4
	
	int check_switches()
	{
	int modesel=0;
	if (PINC & (1<<3)) 
	{
		modesel=modesel+1;
	
	} 
	if (PINC & (1<<5))
	{
		modesel=modesel+2;
	}
	//return value:
	///////PINC== 00     01      10     11
	////modesel== 0      1       2      3

	return modesel;
	
	
	}
	
	//////////////////
	//check_button()//
	//////////////////
	
	//Description: Checks the button
	//debouncing should be done in hardware
	//button is on C.2

	int check_button()
	{
		int but=0;
		if(PINC&(1<<7))
		{
			but=1;
		}
		return but;
	}
	
	/////////////////
	//update_LEDs()//
	/////////////////
	
	//Description: Pass the angle and update the LEDs
	//LEDs are on B.3 and B.4

	void update_LEDs(int angle)
	{
		//the function for the PWM value
		if(angle>180)
			angle=180;
		if(angle<30)
			GPWM=(int)(angle*(255/30));
		else
			GPWM=255;
		if(angle>15)
			RPWM=(int)(255-(angle*255/180));
	}
	
	//threebeeps is a BLOCKING function that plays 3 beeps,
	//it is used almost solely for it's blocking purposes, but
	//also lets the user know that the controller is currently doing
	//nothing but beeping  (used for compass calibration)
	void threebeeps()
	{
		play_sound(500,0);
		while(soundcnt>0){	
			RPWM=255;
		}
		play_sound(500,512);
		while(soundcnt>0){	
			RPWM=0;
			GPWM=255;				
		}
		play_sound(500,0);
		while(soundcnt>0){	
			RPWM=255;
		}
		play_sound(500,512);
		while(soundcnt>0){	
			RPWM=0;
			GPWM=255;				
		}
		play_sound(500,0);
		while(soundcnt>0){	
			RPWM=255;
		}
		play_sound(500,512);
		while(soundcnt>0){	
			RPWM=0;
			GPWM=255;				
		}
		play_sound(500,0);
		while(soundcnt>0){	
			RPWM=255;
		}
	}
