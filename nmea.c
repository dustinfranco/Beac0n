#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "nmea.h"
#include <math.h>
#include <float.h>
//set up ASSERT

/*#ifndef __ASSERT_USE_STDERR
#define __ASSERT_USE_STDERR
#include <assert.h>
#endif
*/

#define NMEA_BUF_SIZE 500  //  4 sentences * 80 chars = 320chars
char NmeaBuffer[NMEA_BUF_SIZE]; //UART Rx circular buffer
volatile unsigned int rx_in; //buffer put cursor
volatile unsigned int rx_out; //buffer get cursor

#define NMEA_SENTENCE 81
char Nmea_GGA[NMEA_SENTENCE]; //most recent GGA
char Nmea_RMC[NMEA_SENTENCE]; //most recent RMC
char Nmea_GSA[NMEA_SENTENCE]; //most recent GSA

//super-internal variables
char Nmea_Temp[NMEA_SENTENCE]; //temp sentence processing buffer
int tempCur; //cursor into Nmea_temp
char startFlag; //signals that we have seen a $ to start a sentence

float lat; //most recent latitude
float lon; //most recent longitude
char valid; //fix validity
char sats; //number of visible satellites

//DANGEROUS EXPERIMENT
float stof(const char* s){
  float rez = 0, fact = 1;
  if (*s == '-'){
    s++;
    fact = -1;
  };
  for (int point_seen = 0; *s; s++){
    if (*s == '.'){
      point_seen = 1; 
      continue;
    };
    int d = *s - '0';
    if (d >= 0 && d <= 9){
      if (point_seen) fact /= 10.0f;
      rez = rez * 10.0f + (float)d;
    };
  };
  return rez * fact;
};


void initGPS() {
	rx_in = 0;
	rx_out = 0;
	lat = lon = 0.0;
	valid = 0;
}

void NmeaPutChar(char r_char) {
	//Received a character from receiver
    //Queue it onto the circular buffer if possible
	unsigned int i = rx_in + 1;
	if (i == NMEA_BUF_SIZE) i = 0;
	//assert(i != rx_out);
	NmeaBuffer[rx_in] = r_char;
	rx_in = i;
}

char NmeaGetChar() {
	//Gets the next character off the buffer
	//**** RETURNS 0 IF NO BUFFERED CHARACTERS ****
	char retVal;
	if (rx_in == rx_out) {
		retVal = 0;
	}
	else {
		retVal = NmeaBuffer[rx_out];
		//if (retVal == 0) fprintf(stdout, "\r\nPULL 0! RX_IN: %d RX_OUT: %d\r\n", rx_in, rx_out);
		//assert(retVal);
		if(++rx_out == NMEA_BUF_SIZE) rx_out = 0;
	}
	return retVal;
}

void NmeaProcessTask() {
	//Parse messages out of the buffer
	//Needs to be called by a scheduler
			//printf("xdxdxd");
	while (rx_out != rx_in) {
		//Looking for a new sentence

		if (!startFlag) {
			if (NmeaGetChar() == '$') { 
				//Got one, start parsing
				startFlag = 1;
				tempCur = 0;
				Nmea_Temp[tempCur++] = '$';
			}
		}
		
		//Currently processing a sentence
		else {
			//assert(tempCur < (NMEA_SENTENCE - 1));
			char tempChar = NmeaGetChar();
			
			if (tempChar == 0) 
			{
				//buffer is empty, pop out and finish on next execution
				return;
			}
			//put new character into the temp sentence
			Nmea_Temp[tempCur++] = tempChar;
			//Look for end of sentence and process
			if (Nmea_Temp[tempCur-1] == '\n') {
				Nmea_Temp[tempCur] = 0;
				startFlag = 0;
				tempCur = 0;
				//fprintf(stdout, "Process: %s\r\n", Nmea_Temp);
				NmeaProcessSentence(Nmea_Temp);
			}
		}
	}
}

void NmeaProcessSentence(char* sentence) {
	//This function takes a sentence, and determines its type
	//and performs processing if desired
	if (0 == strncmp(sentence, "$GPGGA", 6)) {
		strncpy(Nmea_GGA, sentence, NMEA_SENTENCE);
		Nmea_GGA[NMEA_SENTENCE - 1] = 0;
		ProcessGGA();
	}
	else if (0 == strncmp(sentence, "$GPGSA", 6)) {
		strncpy(Nmea_GSA, sentence, NMEA_SENTENCE);
	//	Nmea_GSA[NMEA_SENTENCE-1] = 0;
	}
	else if (0 == strncmp(sentence, "$GPRMC", 6)) {
		strncpy(Nmea_RMC, sentence, NMEA_SENTENCE);
		Nmea_RMC[NMEA_SENTENCE-1] = 0;
	//	ProcessRMC();
	}
	else if (0 == strncmp(sentence, "$PRWIZCH", 8)) { }
	else {
		//assert(0);
	}
}

void ProcessGGA() {
	//$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
	//Parse latitude, longitude, and fix validity from the GGA
	//char latDir, lonDir;
	int result;
	char time[80];
	char garbage[80];
	
	char latStr[20], lonStr[20];
	
	float latTemp, latMinutes, lonTemp, lonMinutes;

	//fprintf(stdout, "ProcessGGA: %s\n\r", Nmea_GGA);

	char latDirStr[5], lonDirStr[5], validStr[5], satsStr[5];
	result = sscanf(Nmea_GGA, "$GPGGA,%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^','],%[^',']", 
					time, latStr, latDirStr, lonStr, lonDirStr, validStr, satsStr, garbage); 
	if (result) {
		//for ,4807.038,N,01131.000,E
        // Lat is 48deg, 7.038' N
		// Divide by 100, and convert fraction from base 60
		latTemp = atof(latStr)/100.0;
		latMinutes = modf (latTemp, &lat);
		lat = lat + latMinutes*100/60;
		
		lonTemp = atof(lonStr) / 100;
		lonMinutes = modf (lonTemp, &lon);
		lon = lon + lonMinutes*100/60;
		
		valid = atoi(validStr);
		sats = atoi(satsStr);
		if (latDirStr[0] == 'S') lat = lat * -1;
		if (lonDirStr[0] == 'W') lon = lon * -1;
	
	}
	else {
		valid = 0;
	}
	#ifdef DEBUG
	//fprintf(stdout, "Processed GGA Result %d: %s %d %d", result, time, (int)lat, (int)lon);
	#endif
	//fprintf(stdout,"LAT:%f LON:%f\n\r",lat,lon);
}
/*
void ProcessRMC() {
	//Get the time and date out of an RMC
	//$GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68
    //Time: 225446
	//Date: 19/11/94
	char ghour, gmin, gsec, gday, gmonth, gyear;
	int result;
	result = sscanf(Nmea_RMC, "$GPRMC,%2hd%2hd%2hd,%*[^,],%*[^,],%*[^,],%*[^,],%*[^,],%*[^,],%*[^,],%2hd%2hd%2hd", 
		&ghour, &gmin, &gsec, &gday, &gmonth, &gyear);
	if (result > 0) {
		gpsTime.hour = ghour;
		gpsTime.minute = gmin;
		gpsTime.second = gsec;
		gpsTime.day = gday;
		gpsTime.month = gmonth;
		gpsTime.year = (int)gyear + 2000;
	}
	else {
		#ifdef DEBUG
		fprintf(stdout, "Failed to parse RMC: %d", result);
		#endif
	}

}*/

// Functions to return internal variables to outside callers
/*
float GPSGetLatitude() {
	return lat;
}

float GPSGetLongitude() {
	return lon;
}

char GPSIsValid() {
	return valid;
}

char GPSDataReady() {
	return 1; //TODO: MAKE THIS A REAL FUNCTION
}

char* GetNmea_GGA() {
	return Nmea_GGA;
}

char* GetNmea_GSA() {
	return Nmea_GSA;
}

char* GetNmea_RMC() {
	return Nmea_RMC;
}
	
*/
