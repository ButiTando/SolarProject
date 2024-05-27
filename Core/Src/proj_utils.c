/*
 * proj_utils.c
 *
 *  Created on: Mar 3, 2024
 *      Author: I know
 *  	Version: 1.0.0.1
 */

#include "proj_utils.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

extern uint16_t LED3;
progStates* state;
tempSensor* tempSense;
lcdScreen* screen;
powerSensor* powerMeasure;
dateTime* sDateTime;
calibValue* calibVals;
timeMenu* tMenu;

void progInit(progStates* state){

	state->mode = SMODE_SP;
	state->modeState = IDEAL;
	state->targetLed = D2_Pin;
	state->tickLedMax = D2_TICKS;
	resetStr(state->commandMsg,sizeof(state->commandMsg));
	resetStr(state->response,sizeof(state->response));
	state->buttonEnabled = 1;
	state->uartEnabled = 1;
	HAL_GPIO_WritePin(GPIOB, D4_Pin, GPIO_PIN_SET);
}

void tempSensorInit(tempSensor* tempSense){
	tempSense->startTime = HAL_GetTick();
	tempSense->numSamples = (uint32_t) (sizeof(tempSense->digitalSamples) / sizeof(tempSense->digitalSamples[0]));
}

void calibIinit(calibValue* calibVal){

	calibVal->beta = -0.004;
	calibVal->tSTC = 25;

}

void enviroment_resp(progStates* state, tempSensor* tempMeasure){

#if 1

	if(state->mode == SMODE_CALB){
		uint32_t messageLen = 18;
		char tempStr[messageLen];
		sprintf(tempStr,"&_%03d_%03d_%05d_*\n", tempMeasure->taCPMM, tempMeasure->tspCPMM, tempMeasure->luxPMM );
		strcpy(state->response, tempStr);
		state->responseSize = messageLen;
//		state->responseReady = 1;
	}

	else{
		uint32_t messageLen = 18;
		char tempStr[messageLen];
		sprintf(tempStr,"&_%03d_%03d_%05d_*\n", tempMeasure->taC, tempMeasure->tspC, tempMeasure->lux );
		strcpy(state->response, tempStr);
		state->responseSize = messageLen;
		state->responseReady = 1;
	}


#endif

#if 0
	uint32_t messageLen = 16;
	char tempStr[messageLen];
	char taString[3];
	char tspString[3];
	char lxdString[3];

	int2str(ta,taString);
	int2str(tsp,tspString);
	int2str(lxd,lxdString);

	sprintf(tempStr,"&_%s_%s_%s_*\n", taString, tspString, lxdString);

	strcpy(state->response, tempStr);
	state->responseSize = messageLen;
	state->responseReady = 1;
#endif
}

void solarMeasure_resp(progStates* state, powerSensor* powerMeasure){
	state->responseSize = 21;

	if(state->mode == SMODE_CALB){
		char tempStr[22] = "";
		sprintf(tempStr, "&_%04d_%03d_%03d_%03d_*\n", (int)powerMeasure->vPMM, (int)powerMeasure->iPMM, (int)powerMeasure->PMM, (int)powerMeasure->efficiency);
		strcpy(state->response,tempStr);
//		state->responseReady = 1;
	}

	else{
		char tempStr[22] = "";
		sprintf(tempStr, "&_%04d_%03d_%03d_%03d_*\n", (int)powerMeasure->vPMM, (int)powerMeasure->iPMM, (int)powerMeasure->PMM, (int)powerMeasure->efficiency);
		strcpy(state->response,tempStr);

		state->responseReady = 1;
	}


}

void int2str(int number, char *locationOfValue){

#if 1

	if(number>999){
		if(number > 99999){

		}
		else{

		}
		sprintf(locationOfValue,"%d",999);
	}

	else{
		sprintf(locationOfValue,"%03d",number);
	}

#endif

#if 0
	if(number < 10){
		sprintf(locationOfValue, "00%d", number);
	}

	else if (number >= 10 && number < 100){
		sprintf(locationOfValue, "0%d", number);
	}

	else{
		sprintf(locationOfValue, "%d", number);
	}
#endif

}

void setAdcChannel(uint32_t channel, ADC_HandleTypeDef* hadc1){

	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
	if (HAL_ADC_ConfigChannel(hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void EN_Measure(progStates* state, tempSensor* temp, lcdScreen* screen){

	if (state->mode != SMODE_EN){
		HAL_GPIO_WritePin(GPIOB, D2_Pin, 0);
		HAL_GPIO_WritePin(GPIOB, D4_Pin, GPIO_PIN_SET);
//		screen->screenClear = 1;
//		screen->currentDisplay = 2;
		screen->updateScreen = 1;
		state->ledFlash = 1;
		state->targetLed = LED3;
		state->tickLedMax = D3_TICKS;
		state->ticks = 0;
		state->mode = SMODE_EN;
		state->modeState = MEASURE;
		state->commandSize = 0;
	}

	else{
		switch (state->modeState){

		case IDEAL: //Button pressed again and entering Measure mode
			state->ledFlash = 1;
			state->targetLed = LED3;
			state->modeState = MEASURE;
			state->tickLedMax = D3_TICKS;
			state->modeState = MEASURE;
//			screen->updateScreen = 1;
			screen->currentDisplay = 2;
			break;

		case MEASURE: //Button pressed again. Ending measure transmitting then going to ideal.
			taToTemp(tempSense);
			tspToTemp(tempSense);
			toLuxTrue(tempSense);
			enviroment_resp(state, temp);
			setDisplayMode2(screen, temp);
			screen->updateScreen = 1;
			screen->currentDisplay = 2;

			state->ledFlash = 0;
			state->modeState = IDEAL;
			if(state->buttonEnabled){
				state->uartEnabled = 1;
			}

			else{
				state->buttonEnabled = 1;
			}
			break;
		default:
			__NOP();
			break;

		}
	}
}

void SP_Measure(progStates* state, powerSensor* powerMeasure, calibValue* calibVal, lcdScreen* screen){
	if (state->mode != SMODE_SP){
		HAL_GPIO_WritePin(GPIOB, D3_Pin, 0); // Turn off previous state's led.
		HAL_GPIO_WritePin(GPIOB, D4_Pin, GPIO_PIN_SET);
		state->ledFlash = 1;
		state->targetLed = D2_Pin;
		state->tickLedMax = D2_TICKS;
		state->ticks = 0;
		state->mode = SMODE_SP;
		state->modeState = MEASURE;
		state->commandSize = 0;
		powerMeasure->isMeasure = 0;
		powerMeasure->PMM = 0;
		powerMeasure->vPMM = 0;
		powerMeasure->iPMM = 0;
//		screen->screenClear = 1;
		screen->updateScreen = 1;
//		screen->currentDisplay = 1;
	}

	else{
		switch (state->modeState){

		case IDEAL: //Button pressed again and entering Measure mode
			state->ledFlash = 1;
			state->modeState = MEASURE;
			screen->updateScreen = 1;
			powerMeasure->isMeasure  = 1;
			powerMeasure->PMM = 0;
			powerMeasure->vPMM = 0;
			powerMeasure->iPMM = 0;
			TIM2->CCR1 = 39;
//			screen->currentDisplay = 1;
			break;

		case MEASURE: //Button pressed again. Ending measure transmitting then going to ideal.
//			Prepare uart response.

			solarMeasure_resp(state, powerMeasure);
			screen->updateScreen = 1;
			screen->currentDisplay = 1;
			state->ledFlash = 0;
			state->modeState = IDEAL;

			double pNormDenominator = 1.0 + calibVals->beta*((double)tempSense->tspC - calibVals->tSTC);
		    double pNorm = (double) powerMeasure->PMM /  pNormDenominator;
		    double pNormalized = pNorm * (calibVals->luxCalib/ tempSense->lux);
		    uint32_t efficiency = (uint32_t) ((pNormalized/calibVals->mppCalib)*100.0);
		    powerMeasure->efficiency = efficiency;

			if(state->buttonEnabled){
				state->uartEnabled = 1;
			}

			else{
				state->buttonEnabled = 1;
			}
			powerMeasure->isMeasure  = 0;
			setDisplayMode1(screen, powerMeasure);
			break;

		default:
			__NOP();
			break;

		}
	}

}

void CA_Measure(calibValue* calibVal, progStates* state, powerSensor* powerMeasure, tempSensor* tempMeasure, lcdScreen* screen, ADC_HandleTypeDef* hadc, UART_HandleTypeDef* huart){
	if(state->startedCalib == 0){

		powerMeasure->iPMM = 0;
		powerMeasure->vPMM = 0;
		powerMeasure->PMM = 0;

		taToTemp(tempMeasure);
		tspToTemp(tempMeasure);
		toLuxTrue(tempMeasure);

		setAdcChannel(ADC_CHANNEL_10, hadc);
		HAL_ADC_Start(hadc);
		HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
		powerMeasure->voltageRaw = HAL_ADC_GetValue(hadc);
		HAL_ADC_Stop(hadc);

		setAdcChannel(ADC_CHANNEL_4, hadc);
		HAL_ADC_Start(hadc);
		HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
		powerMeasure->currentVoltageRaw = HAL_ADC_GetValue(hadc);
		HAL_ADC_Stop(hadc);

		toPVMeasures(powerMeasure, tempMeasure);



		state->pwmARR = 39;
		state->ledFlash = 1;
		state->targetLed = D4_Pin;
		state->tickLedMax = D4_TICKS;
		state->calPWMIncrTimeStamp = HAL_GetTick();
		state->calStartTime = HAL_GetTick();
		state->startedCalib = 1;
		TIM2->CCR1 = 39;
		state->pwmCRR = 39;

	}

	else{
		if( (HAL_GetTick() - state->calStartTime) < 10000 ){
			if(state->pwmCRR <= 0){
				//Do Efficiency calculations
				state->modeState = IDEAL;
				state->pwmCRR = 1;
				state->uartEnabled = 1;
				state->buttonEnabled = 1;
				state->startedCalib = 0;
				state->ledFlash = 0;
				TIM2->CCR1 = state->pwmCRR;

				calibVal->luxCalib = tempSense->luxPMM;
				calibVal->mppCalib = powerMeasure->PMM;

				enviroment_resp(state, tempMeasure);
				HAL_UART_Transmit_IT(huart,(uint8_t*) state->response, state->responseSize);

				while(huart->gState == HAL_UART_STATE_BUSY_TX){
					__NOP();
				}

				solarMeasure_resp(state, powerMeasure);
				HAL_UART_Transmit_IT(huart,(uint8_t*) state->response, state->responseSize);

			}

			else{

				if((HAL_GetTick() - state->calPWMIncrTimeStamp) < 128){
					taToTemp(tempMeasure);
					tspToTemp(tempMeasure);
					toLuxTrue(tempMeasure);

					setAdcChannel(ADC_CHANNEL_10, hadc);
					HAL_ADC_Start(hadc);
					HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
					powerMeasure->voltageRaw = HAL_ADC_GetValue(hadc);
					HAL_ADC_Stop(hadc);

					setAdcChannel(ADC_CHANNEL_4, hadc);
					HAL_ADC_Start(hadc);
					HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
					powerMeasure->currentVoltageRaw = HAL_ADC_GetValue(hadc);
					HAL_ADC_Stop(hadc);

					toPVMeasures(powerMeasure, tempMeasure);

					if(powerMeasure->PMM < powerMeasure->power){
						powerMeasure->vPMM =  powerMeasure->solarVoltage;
						powerMeasure->iPMM =  powerMeasure->solarCurrent;
						powerMeasure->PMM = powerMeasure->power;

						tempMeasure->luxPMM = tempMeasure->lux;
						tempMeasure->taCPMM = tempMeasure->taC;
						tempMeasure->tspCPMM = tempMeasure->tspC;


					}

					else{
						__NOP();
					}
				}

				else{
//					Changing the PWM duty cycle
					state->pwmCRR = state->pwmCRR - 1;
					TIM2->CCR1 = state->pwmCRR;
					state->calPWMIncrTimeStamp = HAL_GetTick();
				}
			}
		}
		else{
			state->modeState = IDEAL;
			state->pwmCRR = 1;
			state->startedCalib = 0;
			state->ledFlash = 0;
			state->uartEnabled = 1;
			state->buttonEnabled = 1;
			TIM2->CCR1 = state->pwmCRR;

		}
	}
}

void uart_command_set(progStates* state){
//	Extract the required command.
	char command[2] = "";
	char EN[2] = "EN";
	char SP[2] = "SP";
	char CA[2] = "CA";

	command[0] = (char)state->commandMsg[2];
	command[1] = (char)state->commandMsg[3];


#if 0
	int commandIndex = 0;
	for (int i = 0; i < state->commandSize; i++){
		char check = (char)state->commandMsg[i];
		if (check != 38 && //And character
				check != 95 &&//Underscore character
				check != 42 &&// star scharacter
				check != 10)// newline character
		{
			command[commandIndex] = state->commandMsg[i];
			commandIndex++;
		}
	}
#endif


#if 1
	if(!strncmp(command,EN,2)){
		state->cmdCall = SMODE_EN;
	}

	else if(!strncmp(command,SP,2)){
		state->cmdCall = SMODE_SP;
	}

	else if(!strncmp(command,CA,2)){
		state->cmdCall = SMODE_CALB;
	}

	else{
		__NOP();
	}
#endif
	state->commandReady = 0;
	resetStr(state->commandMsg,sizeof(state->commandMsg));
	state->commandSize = 0;

}

void tspToTemp(tempSensor* tempMeasure){
	float tempPulses = (float) intAverage(tempMeasure->digitalSamples, tempMeasure->numSamples);
	float temp = tempPulses/4096.0;
	temp = temp * 256;
	temp  = temp - 50;
	tempMeasure->tspC = temp;
}

void taToTemp(tempSensor* tempMeasure){
	double tempAdcMeasure = (double) intAverage(tempMeasure->analogSamples, tempMeasure->numSamples);
	double temp = tempAdcMeasure;
	temp = temp * (22.0/273.0);
	temp = temp - 273.0;
	tempMeasure->taC = (uint32_t) temp;
}

void toLuxTrue(tempSensor* tempMeasure){
//	Equation Vout = (LUX)^(1.01337) * (10)^(-1.3299) * Rp * 10^(-6)
//	Design LUX = 500000; make LUX subject of equation
#if 1 //My attempt
	double Rref = 11000.0;
	double luxAdcMeasure = (double) intAverage(tempMeasure->luxSamples, tempMeasure->numSamples);
	double Vout = (luxAdcMeasure / 4095.0) * 3.3;
	double exponent = pow(1.013366679, -1.0);
	double denominator = Rref * pow(10.0, -7.13298628);;
	double base = Vout / denominator;
	double tempLux = pow(base,exponent);
#endif

#if 0 // Friends help
	double Rref = 11000.0;
	double luxAdcMeasure = (double) intAverage(tempMeasure->luxSamples, tempMeasure->numSamples);
	double Vout = (luxAdcMeasure / 4095.0) * 3.3;
	double tempLux = (((Vout/Rref)-(0.02*(0.000001)))/(8.02*(0.00000001)));
#endif

	tempMeasure->lux = (uint32_t) fmin(tempLux, 99999.0);
}

void toVoltageMeasure(powerSensor* powerMeasure){
//	This function converts the adc value to the measured voltages.

	double adcResolution = 4095.0;

	double voltageMeasure = (double) powerMeasure->voltageRaw * 3.3;
	double currentVoltageMeasure = (double) powerMeasure->currentVoltageRaw * 3.3;

	voltageMeasure = voltageMeasure / adcResolution;
	currentVoltageMeasure =  currentVoltageMeasure / adcResolution;

	powerMeasure->voltage = (voltageMeasure);
	powerMeasure->currentVoltage = (currentVoltageMeasure);

}

void toPVMeasures(powerSensor* powerMeasure, tempSensor* tempMeasure){

	toVoltageMeasure(powerMeasure);

	double R1 = 13400.0;
	double R2 = 6600.0;
	double R3 = 13220.0;
	double R4 = 6600.0;
	double Rsense = 10.0;

	double vOutSolar = powerMeasure->voltage * (R3 + R4);
	vOutSolar = (vOutSolar / R4);

	double vk = powerMeasure->currentVoltage * (R1 + R2);
	vk = vk / R2 ;
	double solarCurrent = (vOutSolar - vk) / Rsense;
	double vOutSolarMV = vOutSolar * 1000.0;
	double solarCurrentMA = solarCurrent * 1000.0;
	double power = (solarCurrent * vOutSolar) * 1000.0;

	powerMeasure->solarVoltage = (uint32_t) fmin(vOutSolarMV, 9999.0);
	powerMeasure->solarCurrent = (uint32_t) fmin(solarCurrentMA, 999.0);
	powerMeasure->power = (uint32_t) fmin(power, 999.0);

#if 0
	if(vOutSolarMV >= 10000.0){
		powerMeasure->solarVoltage = (uint32_t) 9999.0;
	}
	else{
		powerMeasure->solarVoltage = (uint32_t) vOutSolarMV;
	}

	if(solarCurrentMA >= 1000.0){
		powerMeasure->solarCurrent = (uint32_t) 999.0;
	}
	else{
		powerMeasure->solarCurrent = (uint32_t) solarCurrentMA;
	}

	if(power >= 1000.0){
		powerMeasure->power = (uint32_t) 999.0;
	}
	else{
		powerMeasure->power = (uint32_t) power;
	}
#endif

	capturePMM(powerMeasure, tempMeasure);

}

void capturePMM(powerSensor* powerMeasure, tempSensor* tempMeasure){
	if (powerMeasure->power > powerMeasure->PMM){
		powerMeasure->vPMM = powerMeasure->solarVoltage;
		powerMeasure->iPMM = powerMeasure->solarCurrent;
		powerMeasure->PMM = powerMeasure->power;
		powerMeasure->mppDutyCycle = TIM2->CCR1;
		tempMeasure->taCPMM = tempMeasure->taC;
		tempMeasure->tspCPMM = tempMeasure->tspC;
		tempMeasure->luxPMM = tempMeasure->lux;
	}
	else{
		__NOP();
	}
}
//LCD functions
void screenInit(lcdScreen* screen, tempSensor* tempMeasure, powerSensor* powerMeasure, dateTime* dateTime){
	setDisplayMode1(screen, powerMeasure);
	setDisplayMode2(screen, tempMeasure);
	setDisplayMode3(screen, dateTime);
}

void setDisplayMode1(lcdScreen* screen, powerSensor* powerMeasure){
	if(powerMeasure->isMeasure == 1){
		snprintf(screen->displayMode1[0],sizeof(screen->displayMode1[0]) ,"V:%04dmV  I:%03dmA", (int)powerMeasure->solarVoltage, (int)powerMeasure->solarCurrent);
		snprintf(screen->displayMode1[1], sizeof(screen->displayMode1[0]), "P: %03dmW E:%03d%%", (int)powerMeasure->PMM, (int)powerMeasure->efficiency);
	}
	else{
		snprintf(screen->displayMode1[0], sizeof(screen->displayMode1[0]), "V:%04dmV I:%03dmA", (int)powerMeasure->vPMM, (int)powerMeasure->iPMM);
		snprintf(screen->displayMode1[1], sizeof(screen->displayMode1[0]), "P: %03dmW E:%03d%%", (int)powerMeasure->PMM, (int)powerMeasure->efficiency);
	}

}

void setDisplayMode2(lcdScreen* screen, tempSensor* tempMeasure){
	snprintf(screen->displayMode2[0], sizeof(screen->displayMode1[0]),"AMB:%03dC SP:%03dC", tempMeasure->taC, tempMeasure->tspC);
	snprintf(screen->displayMode2[1], sizeof(screen->displayMode1[0]),"LUX:%05d      ",tempMeasure->lux);
}

void setDisplayMode3(lcdScreen* screen, dateTime* dateTime){
	RTC_DateTypeDef date = dateTime->Date;
	RTC_TimeTypeDef time = dateTime->Time;
	sprintf(screen->displayMode3[0],"%02d/%02d/%04d      ",(int) date.Date, (int)date.Month,(int) dateTime->year );
	sprintf(screen->displayMode3[1],"%02d:%02d:%02d        ",(int) time.Hours,(int) time.Minutes,(int) time.Seconds);
}

void displayMode1(Lcd_HandleTypeDef* lcd, lcdScreen* screen){
	Lcd_cursor(lcd, 0,0);
	Lcd_string(lcd, screen->displayMode1[0]);
	Lcd_cursor(lcd, 1,0);
	Lcd_string(lcd, screen->displayMode1[1]);
	screen->updateScreen = 1;
	screen->currentDisplay = 1;
}

void displayMode2(Lcd_HandleTypeDef* lcd, lcdScreen* screen){
	Lcd_cursor(lcd, 0,0);
	Lcd_string(lcd, screen->displayMode2[0]);
	Lcd_cursor(lcd, 1,0);
	Lcd_string(lcd, screen->displayMode2[1]);
	screen->updateScreen = 1;
	screen->currentDisplay = 2;
}

void displayMode3(Lcd_HandleTypeDef* lcd, lcdScreen* screen){
	Lcd_cursor(lcd, 0,0);
	Lcd_string(lcd, screen->displayMode3[0]);
	Lcd_cursor(lcd, 1,0);
	Lcd_string(lcd, screen->displayMode3[1]);
	screen->currentDisplay = 3;
	screen->updateScreen = 1;
}

void switchScreen(lcdScreen* screen){

	switch (screen->currentDisplay){

	case 1:
		screen->currentDisplay = 2;
//		screen->screenClear = 1;
		screen->updateScreen = 1;
		break;

	case 2:
		screen->currentDisplay = 3;
//		screen->screenClear = 1;
		screen->updateScreen = 1;
		break;

	case 3:
		screen->currentDisplay = 1;
//		screen->screenClear = 1;
		screen->updateScreen = 1;
		break;

	default:
		__NOP();
	}
}
/*
 * isCommandValid takes in a program state and checks if the command is valid. If the command is valid it returns 1 and 0 otherwise.
 *
 * It checks if the first element = &, second element = _, fifth element = * and sixth element = \n
 * */
int isCommandValid(progStates* state){
	int commandValid = 0;

	if (state->commandMsg[0] == 38 &&
		state->commandMsg[1] == 95 &&
		state->commandMsg[4] == 95 &&
		state->commandMsg[5] == 42 &&
		state->commandMsg[6] == 10){
		commandValid = 1;
		return commandValid;
	}

	else{
		return commandValid;
	}
}

void resetStr(char* str,int lenStr){
	memset(str,0,lenStr);
}

uint32_t intAverage(uint32_t* elements, uint32_t numElements){
	uint32_t sum = 0;

	for (int i = 0; i < numElements; i++){
		sum = sum + elements[i];
	}

	float average = (float) sum / (float) numElements;

	return (uint32_t) average;
}
