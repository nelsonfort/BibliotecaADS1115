/* Copyright 2017-2018, Eric Pernia
 * All rights reserved.
 *
 * This file is part of sAPI Library.
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

/*==================[inlcusiones]============================================*/

// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
// sAPI header
#include "sapi.h"
#include "ADS1115.h"
#include "stringManipulation.h"
/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/

/*==================[definiciones de datos externos]=========================*/

DEBUG_PRINT_ENABLE;

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

QueueHandle_t colaMsg;
// Prototipo de funcion de la tarea
void myTask( void* taskParmPtr );
void taskTempMeasure( void* taskParmPtr );
/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void)
{
   // ---------- CONFIGURACIONES ------------------------------
   // Inicializar y configurar la plataforma
   boardConfig();

   // UART for debug messages
   debugPrintConfigUart( UART_USB, 9600);
   debugPrintlnString( "Blinky con freeRTOS y sAPI." );

   // Led para dar se�al de vida
   gpioWrite( LED3, ON );

   // Crear tarea en freeRTOS
   xTaskCreate(
      myTask,                     // Funcion de la tarea a ejecutar
      (const char *)"myTask",     // Nombre de la tarea como String amigable para el usuario
      configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
      0,                          // Parametros de tarea
      tskIDLE_PRIORITY+1,         // Prioridad de la tarea
      0                           // Puntero a la tarea creada en el sistema
   );
   xTaskCreate(
		   taskTempMeasure,                     // Funcion de la tarea a ejecutar
         (const char *)"taskTempMeasure",     // Nombre de la tarea como String amigable para el usuario
         configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
         0,                          // Parametros de tarea
         tskIDLE_PRIORITY+1,         // Prioridad de la tarea
         0                           // Puntero a la tarea creada en el sistema
      );

   debugPrintlnString( "Iniciando scheduler" );
   // Iniciar scheduler
   vTaskStartScheduler();

   // ---------- REPETIR POR SIEMPRE --------------------------
   while( TRUE ) {
      // Si cae en este while 1 significa que no pudo iniciar el scheduler
   }

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
   // directamenteno sobre un microcontroladore y no es llamado por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

// ----------------- CON vTaskDelayUntil----------------------------
void myTask( void* taskParmPtr )
{
   // ---------- CONFIGURACIONES ------------------------------
	TickType_t tiempo_inicio_ciclo = xTaskGetTickCount();
	uint8_t delay_on =100;

   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE) {
      // Intercambia el estado del LEDB
      gpioWrite( LED2, HIGH );

      // Envia la tarea al estado bloqueado durante 500ms
      vTaskDelay( delay_on / portTICK_RATE_MS );
      gpioWrite( LED2, LOW);
      delay_on += 100;
      if( delay_on == 1000) delay_on = 0;

      vTaskDelayUntil(&tiempo_inicio_ciclo,1000/ portTICK_RATE_MS);


   }
}
void taskTempMeasure( void* taskParmPtr )
{
   // ---------- CONFIGURACIONES ------------------------------
	uint16_t result;
	char buffer[30];
	int channel;

	TickType_t tiempo_inicio_ciclo = xTaskGetTickCount();

	debugPrintlnString( "Beginning connection to the ADC..." );
	beginConnection();
	debugPrintlnString( "Setting the I2C address of the ADC..." );
	setADC_ADDRESS(ADS1115_I2C_ADDRESS_GND);
	debugPrintlnString( "Setting the GAIN of the PGA of the ADC..." );
	setGain(GAIN_0256);

	debugPrintlnString( "Begin to read the ADC..." );
   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE) {

	   for (channel = 0 ; channel <=0 ; channel++){
		   result = readADC_SingleEnded(channel);

		   debugPrintString( "Result of channel " );
		   debugPrintInt(channel);
		   debugPrintString( ": ");
		   debugPrintString(integerToString(result,buffer,10));
		   debugPrintString( "--- Volts: " );
		   debugPrintString(floatToString(getLastConvertion_voltage(),buffer));
		   debugPrintString( "--- Resistance: " );
		   debugPrintString(floatToString(getLastConvertion_resistance(),buffer));
		   debugPrintString( "--- Temperature: " );
		   debugPrintlnString(floatToString(getLastConvertion_temperature(),buffer));


		   vTaskDelayUntil(&tiempo_inicio_ciclo,1000/ portTICK_RATE_MS);
	   }

   }
}
/*=======[fin del archivo]========================================*/
