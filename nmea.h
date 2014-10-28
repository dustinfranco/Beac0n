#ifndef NMEA_H
#define NMEA_H

struct GPS_TIME {
	int year;
	char month;
	char day;
	char hour;
	char minute;
	char second;
};

struct GPS_TIME gpsTime;

// Routine to append a received character to the internal buffer
void NmeaPutChar(char r_char);

//Remove the oldest character from the interal buffer
char NmeaGetChar();

//Task loop for processing internal buffer. Call by external scheduler
void NmeaProcessTask();

//Internal routine for processing an NMEA sentence removed from the buffer
void NmeaProcessSentence(char* sentence);

void ProcessGGA();
void ProcessRMC();

float GPSGetLatitude();
float GPSGetLongitude();

char GPSIsValid();
char GPSDataReady();
char* GetNmea_GGA();
char* GetNmea_GSA();
char* GetNmea_RMC();

#endif
