/**
 ******************************************************************************
 * @file    main.c
 * @author  Tobias De Gasperis
 *          HAW-Hamburg
 *          Labor für technische Informatik
 *          Berliner Tor  7
 *          D-20099 Hamburg
 * @version 1.0
 *
 * @date    29. April 2022
 * @brief   Rahmen fuer den Zugriff auf das Nucleo IKS01A3-Sensor-Board
 *
 ******************************************************************************
 */

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "init.h"
#include "delay.h"
#include "LCD_GUI.h"
#include "LCD_Demos.h"
#include "lcd.h"
#include "fontsFLASH.h"
#include "LCD_Touch.h"
#include "error.h"
#include "iks01a3.h"
#include "timer.h"

#define NUMBER_OF_AXES 3
#define SIZE_OF_BUFFER 32
#define SIZE_OF_STRING 64
#define DECIMAL_DIGITS 2
#define NUM_OF_SAMPLES 1024
#define NUM_OF_VALUES 13

static int32_t buffer[NUM_OF_SAMPLES][NUM_OF_VALUES]; 

/* Helper function for printing floats & doubles 
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
char *print_double(char *str, double v, int decimalDigits)
{
    int i = 1;
    int intPart, fractPart;
    int len;
    char *ptr;

    /* prepare decimal digits multiplicator */
    for (; decimalDigits != 0; i *= 10, decimalDigits--);

    /* calculate integer & fractinal parts */
    intPart = (int)v;
    fractPart = (int)((v - (double)(int)v) * i);

    /* fill in integer part */
    sprintf(str, "%i.", intPart);

    /* prepare fill in of fractional part */
    len = strlen(str);
    ptr = &str[len];

    /* fill in leading fractional zeros */
    for (i /= 10; i > 1; i /= 10, ptr++) {
        if (fractPart >= i) {
            break;
        }
        *ptr = '0';
    }

    /* fill in (rest of) fractional part */
    sprintf(ptr, "%i", fractPart);

    return str;
}


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void){
	
	IKS01A3_SENSORS_t iks;
	uint8_t id = 0;
	int32_t axes[NUMBER_OF_AXES] = {0};
	float value1 = 0.0f;
	float value2 = 0.0f;
	char string[SIZE_OF_STRING] = {0};
  char buffer1[SIZE_OF_BUFFER] = {0};
	char buffer2[SIZE_OF_BUFFER] = {0};
	int cnt = 0;
	
	initITSboard();                 // Initialisierung des ITS Boards
	GUI_init(DEFAULT_BRIGHTNESS);   // Initialisierung des LCD Boards mit Touch
	TP_Init(false);                 // Initialisierung des LCD Boards mit Touch
	if (!checkVersionFlashFonts()) {
	    // Ueberpruefe Version der Fonts im Flash passt nicht zur Software Version
		Error_Handler();
	}
	
	lcdPrintlnS("Nucleo IKS01A3");
	lcdPrintlnS("");
	lcdPrintlnS("");
	
	acc_gyro.init(&iks.acc_gyro);
	accelerometer.init(&iks.acc);
	magnetometer.init(&iks.mag);
	hum_temp.init(&iks.hum_temp);
	press_temp.init(&iks.press_temp);
	temp.init(&iks.temp);
	
	acc_gyro.enable_acc();
	acc_gyro.enable_gyr();
	accelerometer.enable_acc();
	magnetometer.enable_mag();
	hum_temp.enable_hum();
	hum_temp.enable_temp();
	press_temp.enable_press();
	press_temp.enable_temp();
	temp.enable_temp();


	initTimer();
	while (1) { 
		cnt++;

			GPIOD->ODR = 0x01;//LED
			for(int i = 0; i < NUM_OF_SAMPLES; i++ ){
					acc_gyro.get_a_axes(axes);
					buffer[i][0] = getTimeStamp();
					buffer[i][1] = axes[0];
					buffer[i][2] = axes[1];
					buffer[i][3] = axes[2];
					acc_gyro.get_g_axes(axes);
					buffer[i][4] = axes[0];
					buffer[i][5] = axes[1];
					buffer[i][6] = axes[2];
					accelerometer.get_a_axes(axes);
					buffer[i][7] = axes[0];
					buffer[i][8] = axes[1];
					buffer[i][9] = axes[2];
					magnetometer.get_m_axes(axes);
					buffer[i][10] = axes[0];
					buffer[i][11]= axes[1];
					buffer[i][12]= axes[2];

		initTimer();
		
		GPIOD->ODR = 0x01;//LED
		for(int i = 0; i < NUM_OF_SAMPLES; i++ ){
			acc_gyro.get_a_axes(axes);
			buffer[i][0] = getTimeStamp();
			buffer[i][1] = axes[0];
			buffer[i][2] = axes[1];
			buffer[i][3] = axes[2];
			acc_gyro.get_g_axes(axes);
			buffer[i][4] = axes[0];
			buffer[i][5] = axes[1];
			buffer[i][6] = axes[2];
			accelerometer.get_a_axes(axes);
			buffer[i][7] = axes[0];
			buffer[i][8] = axes[1];
			buffer[i][9] = axes[2];
			magnetometer.get_m_axes(axes);
			buffer[i][10] = axes[0];
			buffer[i][11]= axes[1];
			buffer[i][12]= axes[2];
		}
		
		GPIOD->ODR = 0x00;//LED
		
		
		for(int i = 0; i < NUM_OF_SAMPLES; i++ ){
			for (int n = 0; n < NUM_OF_VALUES-1; n++){
				printf("%6d;", buffer[i][n]);
			}
			printf("%6d\r\n", buffer[i][NUM_OF_VALUES-1]);
		}
	}
	
}

// EOF
