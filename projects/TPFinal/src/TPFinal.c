/* Copyright 2017, Esteban Volentini - Facet UNT, Fi UNER
 * Copyright 2014, 2015 Mariano Cerdeiro
 * Copyright 2014, Pablo Ridolfi
 * Copyright 2014, Juan Cecconi
 * Copyright 2014, Gustavo Muro
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** @file TPFinal.c
 **
 ** @brief Control dos motores PaP
 **
 ** Autores: Hachmann - Neville - Cavallo
 **
 ** El proyecto consta en repetir una trayectoria la cual es programada al mover 
 ** un sistema de dos grados de libertad en forma manual. Tiene dos motores PaP 
 ** como actuadores y dos potenciometros lineales como sensores. Los valores de 
 ** los sensores se adquieren a una tasa de muestreo de 100Hz.
 **
 ** El programa fue propuesto con 5 tareas.
 **
 ** DoSteps: Que envia la secuencia de pasos a los motores PaP
 ** DecodeCMD: Decodifica los comandos enviados desde la PC
 ** SendResponse: Envia las respuestas a la PC
 ** GetValues: Se encarga de adquirir los valores de los ADC SampleRate = 100ms
 ** RunMovment: Recorre los datos adquiridos y los pasa a DoSteps
 **
 ** Las tareas se sincronizan utilizando Eventos. 
 */

/* === Inclusiones de cabeceras ============================================ */
#include "../../TPFinal/inc/FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "led.h"
#include "switch.h"
#include "soc.h"
#include "semphr.h"
#include "event_groups.h"
#include "queue.h"
#include "uart.h"
#include "ctype.h"
#include "string.h"
#include "adc.h"


/* === Definicion y Macros ================================================= */

#define NEXTSTEP		(1<<0)
#define STEPSREADY		(1<<1)
#define GETVALUES		(1<<2)
#define STOPVALUES		(1<<3)
#define RUNNING			(1<<4)


//#define M01(State)		Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 0);

/* === Declaraciones de tipos de datos internos ============================ */

typedef struct{
	char cmd[3][16];
	uint8_t nArg;
} _Command;

typedef struct{
    int32_t     Steps;
    int8_t      StepMode;
    int8_t      index;
    uint8_t     rpm;
    uint8_t		sync;
    int32_t		stepsSync;
    int8_t		stepsRate;
    int16_t		vIntToStpes;
} _StepMotor;


/* === Declaraciones de funciones internas ================================= */

/** @brief Función que implementa una tarea de baliza
 ** 
 ** @parameter[in] parametros Puntero a una estructura que contiene el led
 **                           y la demora entre encendido y apagado.
 */ 
void DecodeCMD(void * parametros);
void DoSteps(void * parametros);
void SendResponse(void * parametros);
void GetValues(void *parametros);
void RunMovement(void *parametros);

void InitMotors();
void EnviarTexto(const char * cadena);
int32_t StrToInt(const char * cadena);
void IntToStr(char *str, int value);

const char SequenceSteps[8]={0x11,0x33,0x22,0x66,0x44,0xCC,0x88,0x99};

EventGroupHandle_t xCreatedEventGroup;
QueueHandle_t xQueueRX, xQueueCMD, xQueueTX;
uint8_t lastKey;
uint32_t maxTimeValues;
uint8_t iMovementData, currentMotor;
uint16_t movementDataM1[200];
uint16_t movementDataM2[200];
_StepMotor SM1, SM2;



/* === Definiciones de variables internas ================================== */

/* === Definiciones de variables externas ================================== */

/* === Definiciones de funciones internas ================================== */
void DoSteps(void *parametros){
	uint8_t doStep;
	BaseType_t uValue;

	while(1){
		xEventGroupWaitBits(xCreatedEventGroup, NEXTSTEP, pdFALSE, pdFALSE, portMAX_DELAY);

		if(SM1.Steps != 0){
			doStep = 1;
			if(SM1.sync){
				if(SM2.stepsSync == SM2.Steps){
					SM2.stepsSync -= SM2.stepsRate;
					if(SM2.stepsSync < 0)
						SM2.stepsSync = 0;
				}
				else
					doStep = 0;
			}

			if(doStep){
				SM1.Steps--;
				SM1.index += SM1.StepMode;
				if(SM1.index < 0)
					SM1.index = 7;
				if(SM1.index > 7)
					SM1.index = SM1.StepMode-1;
				if(SequenceSteps[SM1.index] & 0x01)
					Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 3, 0);
				else
					Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 0);
				if(SequenceSteps[SM1.index] & 0x02)
					Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 3, 3);
				else
					Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 3);
				if(SequenceSteps[SM1.index] & 0x04)
					Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 3, 4);
				else
					Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 4);
				if(SequenceSteps[SM1.index] & 0x08)
					Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 5, 15);
				else
					Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 5, 15);
			}
		}
		if(SM2.Steps != 0){
			doStep = 1;
			if(SM2.sync){
				if(SM1.stepsSync == SM1.Steps){
					SM1.stepsSync -= SM1.stepsRate;
					if(SM1.stepsSync < 0)
						SM1.stepsSync = 0;
				}
				else
					doStep = 0;
			}
			if(doStep){
				SM2.Steps--;
				SM2.index += SM2.StepMode;
				if(SM2.index < 0)
					SM2.index = 7;
				if(SM2.index > 7)
					SM2.index = SM2.StepMode-1;
				if(SequenceSteps[SM2.index] & 0x01)
					Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 5, 16);
				else
					Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 5, 16);
				if(SequenceSteps[SM2.index] & 0x02)
					Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 3, 5);
				else
					Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 5);
				if(SequenceSteps[SM2.index] & 0x04)
					Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 3, 6);
				else
					Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 6);
				if(SequenceSteps[SM2.index] & 0x08)
					Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 3, 7);
				else
					Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 7);
			}
		}

		if(SM1.Steps==0 && SM2.Steps==0){
			Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 0);
			Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 3);
			Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 4);
			Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 5, 15);
			Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 5, 16);
			Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 5);
			Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 6);
			Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 7);
			xEventGroupClearBits(xCreatedEventGroup, NEXTSTEP);
			xEventGroupSetBits(xCreatedEventGroup, STEPSREADY);
			Led_Off(YELLOW_LED);

			uValue = xEventGroupGetBits(xCreatedEventGroup);
			if(!(uValue & RUNNING))
				EnviarTexto("\r\nSTEPS FINISHED\r\n");
		}
		vTaskDelay(2 / portTICK_PERIOD_MS);
	}
}

void DecodeCMD(void * parametros){
	static _Command cmd1 = {"", "", "", 0};
	uint8_t index, lastChar;
	uint8_t buffer[48];
	uint8_t i;
	int16_t readValue, readValue1;
	BaseType_t uValue;

	while(1){
		xQueueReceive(xQueueRX, &buffer[0], portMAX_DELAY);

		index = 0;
		lastChar = '\0';
		cmd1.nArg = 0;

		for(i=0; i<48 && buffer[i]!='\r'; i++){
			if(buffer[i]==' ' && lastChar!=' '){
				cmd1.cmd[cmd1.nArg][index] = '\0';
				index = 0;
				cmd1.nArg++;
				if(cmd1.nArg == 3){
					cmd1.nArg = 0;
					break;
				}
			}
			else{
				cmd1.cmd[cmd1.nArg][index++] = buffer[i];
				if(index == 16)
					index = 0;
			}
			lastChar = buffer[i];
		}
		if(i<48 && buffer[i]=='\r'){
			cmd1.cmd[cmd1.nArg][index] = '\0';
			cmd1.nArg++;
			if(cmd1.nArg == 1){
				if(strcmp("TEST", cmd1.cmd[0]) == 0)
					EnviarTexto("\r\nTEST OK\r\n");
				if(strcmp("STOP", cmd1.cmd[0]) == 0){
					xEventGroupClearBits(xCreatedEventGroup, NEXTSTEP | RUNNING);
					EnviarTexto("\r\nSTOP OK\r\n");
					Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 0);
					Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 3);
					Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 4);
					Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 5, 15);
					Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 5, 16);
					Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 5);
					Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 6);
					Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 7);
					Led_Off(YELLOW_LED);
					Led_Off(GREEN_LED);
				}
				if(strcmp("START", cmd1.cmd[0]) == 0){
					uValue = xEventGroupGetBits(xCreatedEventGroup);
					if(!(uValue & GETVALUES) && !(uValue & NEXTSTEP)){
						if(movementDataM1[0]!=0xFFFF && movementDataM2[0]!=0xFFFF){
							iMovementData = 255;
							xEventGroupSetBits(xCreatedEventGroup, RUNNING);
							Led_On(GREEN_LED);
							EnviarTexto("\r\nSTART OK\r\n");
						}
					}
					else
						EnviarTexto("\r\nSTART ERROR\r\n");
				}
				if(strcmp("STARTGET", cmd1.cmd[0]) == 0){
					uValue = xEventGroupGetBits(xCreatedEventGroup);
					if(!(uValue & NEXTSTEP) && !(uValue & RUNNING)){
						iMovementData = 255;
						currentMotor = 0;
						xEventGroupSetBits(xCreatedEventGroup, GETVALUES);
					}
					else
						EnviarTexto("\r\nSTARTGET ERROR\r\n");
				}
				if(strcmp("STOPGET", cmd1.cmd[0]) == 0){
					xEventGroupSetBits(xCreatedEventGroup, STOPVALUES);
					EnviarTexto("\r\nSTOPGET OK\r\n");
				}
				if(strcmp("GETADC", cmd1.cmd[0]) == 0){
					Chip_ADC_EnableChannel(LPC_ADC0,  ADC_CH2, DISABLE);
					Chip_ADC_EnableChannel(LPC_ADC0,  ADC_CH1, ENABLE);
					Chip_ADC_SetStartMode(LPC_ADC0, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
					while (Chip_ADC_ReadStatus(LPC_ADC0, ADC_CH1, ADC_DR_DONE_STAT) != SET);
					Chip_ADC_ReadValue(LPC_ADC0, ADC_CH1, &readValue);
					IntToStr(buffer, readValue);
					EnviarTexto("\r\n");
					EnviarTexto(buffer);
					EnviarTexto("\r\n");
					Chip_ADC_EnableChannel(LPC_ADC0,  ADC_CH1, DISABLE);
					Chip_ADC_EnableChannel(LPC_ADC0,  ADC_CH2, ENABLE);
					Chip_ADC_SetStartMode(LPC_ADC0, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
					while (Chip_ADC_ReadStatus(LPC_ADC0, ADC_CH2, ADC_DR_DONE_STAT) != SET);
					Chip_ADC_ReadValue(LPC_ADC0, ADC_CH2, &readValue);
					IntToStr(buffer, readValue);
					EnviarTexto("\r\n");
					EnviarTexto(buffer);
					EnviarTexto("\r\n");
				}
			}
			if(cmd1.nArg == 2){
				if(strcmp("READMOVE", cmd1.cmd[0]) == 0){
					uValue = xEventGroupGetBits(xCreatedEventGroup);
					if(uValue & GETVALUES)
						EnviarTexto("\r\nGETTING VAUES\r\n");
					else{
						index = StrToInt(cmd1.cmd[1]);
						EnviarTexto("\r\nM1 ");
						EnviarTexto(cmd1.cmd[1]);
						EnviarTexto(": ");
						IntToStr(buffer, movementDataM1[index]);
						EnviarTexto(buffer);
						EnviarTexto("\r\n");
						EnviarTexto("\r\nM2 ");
						EnviarTexto(cmd1.cmd[1]);
						EnviarTexto(": ");
						IntToStr(buffer, movementDataM2[index]);
						EnviarTexto(buffer);
						EnviarTexto("\r\n");
					}
				}
			}
			if(cmd1.nArg == 3){
				if(strcmp("STEPS01", cmd1.cmd[0]) == 0){
					uValue = xEventGroupGetBits(xCreatedEventGroup);
					if(uValue & NEXTSTEP){
						EnviarTexto("\r\nSTEPS01 BUSY\r\n");
					}
					else{
						SM1.sync = 0;
						SM1.Steps = StrToInt(cmd1.cmd[1]);
						SM1.StepMode = 2;
						if(SM1.Steps < 0){
							SM1.Steps *= -1;
							SM1.StepMode = -2;
						}
						SM2.sync = 0;
						SM2.Steps = StrToInt(cmd1.cmd[2]);
						SM2.StepMode = 2;
						if(SM2.Steps < 0){
							SM2.Steps *= -1;
							SM2.StepMode = -2;
						}
						xEventGroupSetBits(xCreatedEventGroup, NEXTSTEP);
						Led_On(YELLOW_LED);
						EnviarTexto("\r\nSTEPS01 OK\r\n");
					}
				}
				if(strcmp("STEPS02", cmd1.cmd[0]) == 0){
					uValue = xEventGroupGetBits(xCreatedEventGroup);
					if(uValue & NEXTSTEP){
						EnviarTexto("\r\nSTEPS02 BUSY\r\n");
					}
					else{
						SM1.Steps = StrToInt(cmd1.cmd[1]);
						SM1.StepMode = 2;
						if(SM1.Steps < 0){
							SM1.Steps *= -1;
							SM1.StepMode = -2;
						}
						SM2.Steps = StrToInt(cmd1.cmd[2]);
						SM2.StepMode = 2;
						if(SM2.Steps < 0){
							SM2.Steps *= -1;
							SM2.StepMode = -2;
						}
						SM1.sync = 0;
						SM2.sync = 0;
						if(SM1.Steps!=0 && SM2.Steps!=0){
							if(SM1.Steps > SM2.Steps){
								SM2.sync = 1;
								SM1.stepsRate = SM1.Steps/SM2.Steps;
								SM1.stepsSync = SM1.Steps-SM1.stepsRate;
							}
							else{
								if(SM1.Steps < SM2.Steps){
									SM1.sync = 1;
									SM2.stepsRate = SM2.Steps/SM1.Steps;
									SM2.stepsSync = SM2.Steps-SM2.stepsRate;
								}
							}
						}
						xEventGroupSetBits(xCreatedEventGroup, NEXTSTEP);
						Led_On(YELLOW_LED);
						EnviarTexto("\r\nSTEPS02 OK\r\n");
					}
				}
				if(strcmp("SETCAL", cmd1.cmd[0]) == 0){
					readValue = StrToInt(cmd1.cmd[1]);
					readValue1 = StrToInt(cmd1.cmd[2]);
					if(readValue<1023 && readValue>-1023 && readValue!=0)
						SM1.vIntToStpes = readValue;
					if(readValue1<1023 && readValue1>-1023 && readValue1!=0)
						SM2.vIntToStpes = readValue1;
					EnviarTexto("\r\nCalibration Const. 90 -> V\r\n\r\nM1: ");
					IntToStr(buffer, SM1.vIntToStpes);
					EnviarTexto(buffer);
					EnviarTexto(" - M2: ");
					IntToStr(buffer, SM2.vIntToStpes);
					EnviarTexto(buffer);
					EnviarTexto("\r\n");
				}

			}
		}
		else
			EnviarTexto("\r\nCMD ERROR\r\n");
	}
}

void SendResponse(void *parametros) {
	uint8_t buffer;

	while(1){
		if(Chip_UART_ReadLineStatus(USB_UART) & UART_LSR_THRE) {
			xQueueReceive(xQueueTX, &buffer, portMAX_DELAY);

			Chip_UART_SendByte(USB_UART, buffer);
		}
	}
}

void GetValues(void *parametros){
	uint8_t i;
	BaseType_t uValue;

	while(1){
		xEventGroupWaitBits(xCreatedEventGroup, GETVALUES, pdFALSE, pdFALSE, portMAX_DELAY);
		if(iMovementData == 255){
			for(i = 0; i<200; i++){
				movementDataM1[i] = 0xFFFF;
				movementDataM2[i] = 0xFFFF;
			}
			currentMotor = 0;
			maxTimeValues = 300;//15Seg
		}
		if(currentMotor){
			Chip_ADC_ReadValue(LPC_ADC0, ADC_CH1, &movementDataM1[iMovementData]);
			Chip_ADC_EnableChannel(LPC_ADC0, ADC_CH1, DISABLE);
			Chip_ADC_EnableChannel(LPC_ADC0, ADC_CH2, ENABLE);
			currentMotor = 0;
		}
		else{
			maxTimeValues--;
			iMovementData++;
			uValue = xEventGroupGetBits(xCreatedEventGroup);
			if(iMovementData==200 || maxTimeValues==0 || (uValue & STOPVALUES)){
				Led_Off(RED_LED);
				xEventGroupClearBits(xCreatedEventGroup, GETVALUES | STOPVALUES);
			}
			if(iMovementData>0 && iMovementData<200)
				Chip_ADC_ReadValue(LPC_ADC0, ADC_CH2, &movementDataM2[iMovementData-1]);
			Chip_ADC_EnableChannel(LPC_ADC0, ADC_CH2, DISABLE);
			Chip_ADC_EnableChannel(LPC_ADC0, ADC_CH1, ENABLE);
			currentMotor = 1;
		}
		uValue = xEventGroupGetBits(xCreatedEventGroup);
		if(uValue & GETVALUES){
			Chip_ADC_SetStartMode(LPC_ADC0, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
			if(currentMotor)
				Led_Toggle(RED_LED);
			if(maxTimeValues)
				maxTimeValues--;
		}
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

void RunMovement(void *parametros){
	BaseType_t uValue;
	uint16_t valueReadM1, valueReadM2;

	while(1){
		xEventGroupWaitBits(xCreatedEventGroup, RUNNING, pdFALSE, pdFALSE, portMAX_DELAY);
		uValue = xEventGroupGetBits(xCreatedEventGroup);
		if(!(uValue & NEXTSTEP)){
			if(iMovementData == 255){
				Chip_ADC_EnableChannel(LPC_ADC0, ADC_CH2, DISABLE);
				Chip_ADC_EnableChannel(LPC_ADC0,  ADC_CH1, ENABLE);
				Chip_ADC_SetStartMode(LPC_ADC0, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
				while (Chip_ADC_ReadStatus(LPC_ADC0, ADC_CH1, ADC_DR_DONE_STAT) != SET);
				Chip_ADC_ReadValue(LPC_ADC0, ADC_CH1, &valueReadM1);
				Chip_ADC_EnableChannel(LPC_ADC0, ADC_CH1, DISABLE);
				Chip_ADC_EnableChannel(LPC_ADC0,  ADC_CH2, ENABLE);
				Chip_ADC_SetStartMode(LPC_ADC0, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
				while (Chip_ADC_ReadStatus(LPC_ADC0, ADC_CH2, ADC_DR_DONE_STAT) != SET);
				Chip_ADC_ReadValue(LPC_ADC0, ADC_CH2, &valueReadM2);
				SM1.Steps = valueReadM1 - movementDataM1[0];
				SM2.Steps = valueReadM2 - movementDataM2[0];
				iMovementData = 0;
				EnviarTexto("\r\nRUNNING\r\n");
			}
			else{
				iMovementData++;
				if(iMovementData==200 || movementDataM1[iMovementData]==0xFFFF){
					SM1.Steps = movementDataM1[0] - movementDataM1[iMovementData-1];
					SM2.Steps = movementDataM2[0] - movementDataM2[iMovementData-1];
					iMovementData = 0;
					EnviarTexto("\r\nRUNNING\r\n");
				}
				else{
					SM1.Steps = movementDataM1[iMovementData] - movementDataM1[iMovementData-1];
					SM2.Steps = movementDataM2[iMovementData] - movementDataM2[iMovementData-1];
				}
			}
			SM1.Steps *= 512;
			SM1.Steps /= SM1.vIntToStpes;
			SM2.Steps *= 512;
			SM2.Steps /= SM2.vIntToStpes;
			SM1.StepMode = 2;
			SM2.StepMode = 2;
			if(SM1.Steps < 0){
				SM1.Steps *= -1;
				SM1.StepMode = -2;
			}
			if(SM2.Steps < 0){
				SM2.Steps *= -1;
				SM2.StepMode = -2;
			}
			SM1.sync = 0;
			SM2.sync = 0;
			if(SM1.Steps!=0 && SM2.Steps!=0){
				if(SM1.Steps > SM2.Steps){
					SM2.sync = 1;
					SM1.stepsRate = SM1.Steps/SM2.Steps;
					SM1.stepsSync = SM1.Steps-SM1.stepsRate;
				}
				else{
					if(SM1.Steps < SM2.Steps){
						SM1.sync = 1;
						SM2.stepsRate = SM2.Steps/SM1.Steps;
						SM2.stepsSync = SM2.Steps-SM2.stepsRate;
					}
				}
			}
			xEventGroupSetBits(xCreatedEventGroup, NEXTSTEP);
			Led_On(YELLOW_LED);
		}
		vTaskDelay(pdMS_TO_TICKS(20));
	}
}


void EnviarTexto(const char *cadena) {
	uint8_t i = 0;
	BaseType_t value;

	while(cadena[i] != '\0'){
		value = xQueueSendToBack(xQueueTX, &cadena[i], pdMS_TO_TICKS(2));
		if(value == pdPASS)
			i++;
	}
}

int32_t StrToInt(const char * cadena){
	int32_t value = 0;
	int32_t i = 0;

	if(cadena[0] == '-')
		i = 1;
	for(; i<strlen(cadena); i++){
		value *= 10;
		value += (cadena[i] - '0');
	}
	if(cadena[0] == '-')
		value *= -1;

	return value;
}

void IntToStr(char *str, int value){
	int i, j;

	if(value == 0){
		str[0] = '0';
		str[1] = '\0';
		return;
	}
	i = 0;
	j = 0;
	if(value < 0){
		str[0] = '-';
		i = 1;
		j = 1;
        value *= -1;
	}
	for(; i<11 && value!=0; i++){
		str[i] = (value%10 + '0');
		value /= 10;
	}
    str[i] = '\0';
	i--;
	while(i >= j){
		str[11] = str[i];
		str[i] = str[j];
		str[j] = str[11];
        i--;
        j++;
	}
    str[11] = '\0';
}


void InitMotors(){
	Chip_GPIO_Init(LPC_GPIO_PORT);

	/** Mapping of MOTOR1 pins*/
	Chip_SCU_PinMux(6, 1, MD_PUP, FUNC0);
	Chip_SCU_PinMux(6, 4, MD_PUP, FUNC0);
	Chip_SCU_PinMux(6, 5, MD_PUP, FUNC0);
	Chip_SCU_PinMux(6, 7, MD_PUP, FUNC4);

	/** Mapping of MOTOR2 pins*/
	Chip_SCU_PinMux(6, 8, MD_PUP, FUNC4);
	Chip_SCU_PinMux(6, 9, MD_PUP, FUNC0);
	Chip_SCU_PinMux(6, 10, MD_PUP, FUNC0);
	Chip_SCU_PinMux(6, 11, MD_PUP, FUNC0);

	/** Set MOTOR1 port as output*/
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 3, 0);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 3, 3);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 3, 4);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 5, 15);

	/** Set MOTOR2 port as output*/
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 5, 16);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 3, 5);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 3, 6);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 3, 7);

	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 0);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 3);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 4);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 5, 15);

	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 5, 16);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 5);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 6);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 3, 7);

	SM1.index = 1;
	SM1.Steps = 0;
	SM1.StepMode = 2;
	SM1.rpm = 1;
	SM2.index = 1;
	SM2.Steps = 0;
	SM2.StepMode = 2;
	SM2.rpm = 1;

	for(uint8_t i = 0; i<200; i++){
		movementDataM1[i] = 0xFFFF;
		movementDataM2[i] = 0xFFFF;
	}
	SM1.vIntToStpes = 341;//= DV90*1023/3.3 -> DV90: diferencia en volts que corresponden a 90grd.
	SM2.vIntToStpes = 341;

}




/* === Definiciones de funciones externas ================================== */
void UART2_IRQHandler(void) {
	static uint8_t cmd[48], index = 0;
	uint8_t value;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;


	value = Chip_UART_ReadByte(USB_UART);

	if(value == '\r'){
		cmd[index] = value;
		xQueueSendToBackFromISR(xQueueRX, &cmd, &xHigherPriorityTaskWoken);
		index = 0;
	}
	else{
		if(isprint(value)){
			cmd[index++] = value;
			if(index == 48)
				index = 0;
		}
	}
	Led_Toggle(RGB_B_LED);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

void GPIO0_IRQHandler(void)
{
	BaseType_t uValue;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH (0));

	uValue = xEventGroupGetBitsFromISR(xCreatedEventGroup);
	if(uValue & GETVALUES){
		xEventGroupSetBitsFromISR(xCreatedEventGroup, STOPVALUES, &xHigherPriorityTaskWoken);
		Led_Off(RED_LED);
	}
	else{
		if(!(uValue & NEXTSTEP) && !(uValue & RUNNING)){
			iMovementData = 255;
			currentMotor = 0;
			xEventGroupSetBitsFromISR(xCreatedEventGroup, GETVALUES, &xHigherPriorityTaskWoken);
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void GPIO1_IRQHandler(void)
{
	BaseType_t uValue;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH (1));

	uValue = xEventGroupGetBitsFromISR(xCreatedEventGroup);
	if(uValue & RUNNING){
		xEventGroupClearBitsFromISR(xCreatedEventGroup, NEXTSTEP | RUNNING);
		Led_Off(GREEN_LED);
	}
	if(!(uValue & GETVALUES) && !(uValue & NEXTSTEP)){
		if(movementDataM1[0]!=0xFFFF && movementDataM2[0]!=0xFFFF){
			iMovementData = 255;
			xEventGroupSetBitsFromISR(xCreatedEventGroup, RUNNING, &xHigherPriorityTaskWoken);
			Led_On(GREEN_LED);
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}




/** @brief Función principal del programa
 **
 ** @returns 0 La función nunca debería terminar
 **
 ** @remarks En un sistema embebido la función main() nunca debe terminar.
 **          El valor de retorno 0 es para evitar un error en el compilador.
 */
int main(void) {

   /* Inicializaciones y configuraciones de dispositivos */
   //Init_PonchoUNT();
   Init_Leds();
   Init_Switches();
   Init_Uart_Ftdi();
   InitMotors();
   Init_Adc();

   Chip_UART_IntEnable(USB_UART, (UART_IER_RBRINT | UART_IER_RLSINT));
   NVIC_SetPriority(26, 0x1F);
   NVIC_EnableIRQ(26);

	Chip_PININT_Init (LPC_GPIO_PIN_INT);
	Chip_SCU_GPIOIntPinSel(0, 0, 4);
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT , PININTCH (0));
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT , PININTCH (0));
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT , PININTCH (0));

	Chip_SCU_GPIOIntPinSel(1, 0, 8);
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT , PININTCH (1));
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT , PININTCH (1));
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT , PININTCH (1));

	NVIC_ClearPendingIRQ(PIN_INT0_IRQn);
	NVIC_ClearPendingIRQ(PIN_INT1_IRQn);

	NVIC_SetPriority(PIN_INT0_IRQn, 0x1F);
	NVIC_EnableIRQ(PIN_INT0_IRQn);
	NVIC_SetPriority(PIN_INT1_IRQn, 0x1F);
	NVIC_EnableIRQ(PIN_INT1_IRQn);


   xCreatedEventGroup = xEventGroupCreate();

   /* Create the queue, storing the returned handle in the xQueue variable. */
   xQueueRX = xQueueCreate( 2, 48);
   xQueueCMD = xQueueCreate( 2, sizeof(_Command));
   xQueueTX = xQueueCreate( 256, 1);

   /* Creación de las tareas */
   xTaskCreate(DecodeCMD, "DecodeCMD", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL);
   xTaskCreate(DoSteps, "DoSteps", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
   xTaskCreate(RunMovement, "RunMovement", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
   xTaskCreate(GetValues, "GetValues", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
   xTaskCreate(SendResponse, "SendResponse", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

   /* Arranque del sistema operativo */
   SisTick_Init();
   vTaskStartScheduler();
   
   /* vTaskStartScheduler solo retorna si se detiene el sistema operativo */
   while(1);

   /* El valor de retorno es solo para evitar errores en el compilador*/
   return 0;
}


