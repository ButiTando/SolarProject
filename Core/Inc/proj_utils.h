/*
 * proj_utils.h
 *
 *  Created on: Mar 3, 2024
 *      Author: I Know
 */

#include <stdint.h>
#include "main.h"
#include "lcd.h"

#ifndef INC_PROJ_UTILS_H_
#define INC_PROJ_UTILS_H_

typedef struct progState{
	int mode;
	int modeState;
	uint16_t targetLed;
	uint16_t activeBtn;
	int ledFlash;
	int ticks;	// Incremented by timer 3 or Timer linked to leds.
	int tickLedMax; // Compared with with tick to determine when to toggle targetLed. This value can be changed depending on the fleshing frequency of the LED.
	int commandReady;
	char commandMsg[25];
	char commandByte[1];
	int cmdCall;
	int commandSize;
	int responseReady;
	char response[25];
	int responseSize;
	int buttonEnabled; // This is set when a button is click. If zero ignore command.
	int uartEnabled;  // This is set when UART command enabled. If zero ignore command.
	int displayCycle; // If this is zero then the screen should not cycle if 1 cycle.
	int startedCalib;
	int leftPress;
	uint32_t pwmARR;
	uint32_t pwmCRR;
	uint32_t calStartTime;
	uint32_t calPWMIncrTimeStamp;
	int incrCRR;
} progStates;

typedef struct lcdScreens{
//	Each item has two lines.
	int screenRotate;
	int updateScreen;
	int currentDisplay;
	int screenClear;
	int rortateTick;
	char displayMode1[2][16];
	char displayMode2[2][16];
	char displayMode3[2][16];
}lcdScreen;

typedef struct powerSensors{
	uint32_t voltageRawArray[10];
	uint32_t voltageCurrentRawArray[10];
	uint32_t voltageRaw;
	uint32_t currentVoltageRaw;
	double voltage;
	double currentVoltage;
	uint32_t solarVoltage;
	uint32_t solarCurrent;
	uint32_t PMM;
	uint32_t vPMM;
	uint32_t iPMM;
	uint32_t power;
	uint32_t efficiency;
	uint32_t mppDutyCycle;
	int isMeasure;
	int activeLoadStarted;

} powerSensor;

typedef struct tempSensors{
	int digitalBegin; //Flag triggered when the first pulse from the temperature sensor is detected.
	int startTime; // Time stamp of first pulse.
	uint32_t tspRaw; // Pulse count from digital temperature sensor.
	uint32_t taRaw;	// ADC measure from Analog temperature sensor.
	uint32_t luxRaw;
	uint32_t numSamples; // Number of samples to use.
	uint32_t digitalSamples[10]; // Digital temperature pulse count samples.
	uint32_t analogSamples[10];
	uint32_t luxSamples[10];// Analog temperature ADC measure samples.
	int taC; // Analog sensor temperature in degrees celsius.
	int tspC;
	int lux;// Digital sensor temperature in degrees celsius.
	int sampleIndex; // Index within the digitalSamples and analogSamples.
	int taCPMM;
	int tspCPMM;
	int luxPMM;
}tempSensor;

typedef struct bounceStates{
	int level;
	int goLowTime;
	int goHighTime;
} bounceState;

typedef struct dateTimes{
	RTC_TimeTypeDef Time;
	RTC_DateTypeDef Date;
	uint32_t year;
} dateTime;

typedef struct calibValues{
	double luxCalib;
	double mppCalib;
	double beta;
	double tSTC;
	uint32_t mppDutyCycle;
} calibValue;

typedef struct timeMenues{
	int topButtonPressed;
	int bottomButtonPressed;
	int itemIndex;
	int timeSet;
	int timeTick;
}timeMenu;

extern progStates* state; //Declaring this here so it can be used everywhere.
extern tempSensor* tempSense;
extern char emptyResponse[20];
extern int buttonClicked;
extern lcdScreen* screen;
extern powerSensor* powerMeasure;
extern dateTime* sDateTime;
extern calibValue* calibVals;
extern timeMenu* tMenu;
void progInit(progStates* state);
void tempSensorInit(tempSensor* tempSense);
void calibIinit(calibValue* calibVal);
void enviroment_resp(progStates* state, tempSensor* tempMeasure);
void solarMeasure_resp(progStates* state, powerSensor* powerMeasure);
void int2str(int number, char* locationOfValue);
void EN_Measure(progStates* state, tempSensor* tempMeasure, lcdScreen* screen);
void SP_Measure(progStates* state, powerSensor* powerMeasure, calibValue* calibVal, lcdScreen* screen);
void CA_Measure(calibValue* calibVal, progStates* state, powerSensor* powerMeasure, tempSensor* tempMeasure, lcdScreen* screen, ADC_HandleTypeDef* hadc, UART_HandleTypeDef* huart);
void uart_command_set(progStates* state);
void tspToTemp(tempSensor* tempMeasure);
void taToTemp(tempSensor* tempMeasure);
void toLuxTrue(tempSensor* tempMeasure);
void respReset(progStates* state);
void resetStr(char* str,int lenStr);
void setAdcChannel(uint32_t channel, ADC_HandleTypeDef* hadc);
void toVoltageMeasure(powerSensor* powerMeasure);
void toCurrentMeasure(powerSensor* powerMearure);
void toPVMeasures(powerSensor* powerMeasure, tempSensor* tempMeasure);
void capturePMM(powerSensor* powerMeasure, tempSensor* tempMeasure);
void setDisplayMode3(lcdScreen* screem, dateTime* dateTime);

//Display functions
void screenInit(lcdScreen* screen, tempSensor* tempMeasure, powerSensor* powerMeasure, dateTime* dateTime);
void setDisplayMode1(lcdScreen* screen, powerSensor* powerMeasure);
void setDisplayMode2(lcdScreen* screen, tempSensor* tempMeasure);
void setDisplayMode3(lcdScreen* screen, dateTime* dateTime);
void displayMode1(Lcd_HandleTypeDef* lcd, lcdScreen* screen);
void displayMode2(Lcd_HandleTypeDef* lcd, lcdScreen* screen);
void displayMode3(Lcd_HandleTypeDef* lcd, lcdScreen* screen);
void switchScreen(lcdScreen* screen);

int isCommandValid(progStates* state);
uint32_t intAverage(uint32_t* elements, uint32_t numElements);

#endif /* INC_PROJ_UTILS_H_ */
