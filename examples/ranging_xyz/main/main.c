/*******************************************************************************
* Copyright (c) 2020, STMicroelectronics - All Rights Reserved
*
* This file is part of the VL53L5CX Ultra Lite Driver and is dual licensed,
* either 'STMicroelectronics Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, the VL53L5CX Ultra Lite Driver may be distributed under the
* terms of 'BSD 3-clause "New" or "Revised" License', in which case the
* following provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
*******************************************************************************/

/***********************************/
/*   VL53L5CX ULD basic example    */
/***********************************/
/*
* This example is the most basic. It initializes the VL53L5CX ULD, and starts
* a ranging to capture 10 frames.
*
* By default, ULD is configured to have the following settings :
* - Resolution 4x4
* - Ranging period 1Hz
*
* In this example, we also suppose that the number of target per zone is
* set to 1 , and all output are enabled (see file platform.h).
*/

#include <stdlib.h>
#include <string.h>
// #include "vl53l5cx_api.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "vl53l5cx_api.h"
#include "math.h"
#include "plane_algo.h"
/* USER CODE END Includes */
 
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 
/* USER CODE END PTD */
 
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */
 
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
 
/* USER CODE END PM */
 
/* Private variables ---------------------------------------------------------*/
 
/* USER CODE BEGIN PV */
int status;
volatile int IntCount;
uint8_t p_data_ready;
VL53L5CX_Configuration 	Dev;
VL53L5CX_ResultsData 	Results;
uint8_t resolution, isAlive;
uint16_t idx;
 
const double VL53L5_Zone_Pitch8x8[64] = {
		52.00,56.00,61.00,64.00,64.00,61.00,56.00,52.00,
		56.00,62.00,67.00,70.00,70.00,67.00,62.00,56.00,
		61.00,67.00,76.00,78.00,78.00,76.00,67.00,61.00,
		64.00,70.00,78.00,84.00,84.00,78.00,70.00,64.00,
		64.00,70.00,78.00,84.00,84.00,78.00,70.00,64.00,
		61.00,67.00,76.00,78.00,78.00,76.00,67.00,61.00,
		56.00,62.00,67.00,70.00,70.00,67.00,62.00,56.00,
		52.00,56.00,61.00,64.00,64.00,61.00,56.00,52.00
};
 
const double VL53L5_Zone_Yaw8x8[64] = {
		135.00,125.40,113.20, 98.13, 81.87, 66.80, 54.60, 45.00,
		144.60,135.00,120.96,101.31, 78.69, 59.04, 45.00, 35.40,
		156.80,149.04,135.00,108.45, 71.55, 45.00, 30.96, 23.20,
		171.87,168.69,161.55,135.00, 45.00, 18.45, 11.31,  8.13,
		188.13,191.31,198.45,225.00,315.00,341.55,348.69,351.87,
		203.20,210.96,225.00,251.55,288.45,315.00,329.04,336.80,
		203.20,225.00,239.04,258.69,281.31,300.96,315.00,324.60,
		225.00,234.60,246.80,261.87,278.13,293.20,305.40,315.00
};
 
 
PlaneEquation_t PlaneEquation;
XYZ_ZoneCoordinates_t XYZ_ZoneCoordinates;
 
double SinOfPitch[64], CosOfPitch[64], SinOfYaw[64], CosOfYaw[64];
/* USER CODE END PV */
 
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
 
 
 
uint8_t ComputeSinCosTables(void)
{
	//This function will save the math processing time of the code.  If the user wishes to not
	//perform this function, these tables can be generated and saved as a constant.
	uint8_t ZoneNum;
	for (ZoneNum = 0; ZoneNum < 64; ZoneNum++)
	{
		SinOfPitch[ZoneNum] = sin((VL53L5_Zone_Pitch8x8[ZoneNum])*Pi/180);
		CosOfPitch[ZoneNum] = cos((VL53L5_Zone_Pitch8x8[ZoneNum])*Pi/180);
		SinOfYaw[ZoneNum] = sin(VL53L5_Zone_Yaw8x8[ZoneNum]*Pi/180);
		CosOfYaw[ZoneNum] = cos(VL53L5_Zone_Yaw8x8[ZoneNum]*Pi/180);
	}
 
	return 0;
}
 
uint8_t ConvertDist2XYZCoords8x8(VL53L5CX_ResultsData *ResultsData, XYZ_ZoneCoordinates_t *XYZ_ZoneCoordinates)
{
	uint8_t ZoneNum;
	float Hyp;
	for (ZoneNum = 0; ZoneNum < 64; ZoneNum++)
	{
		if ((ResultsData->nb_target_detected[ZoneNum] > 0) && (ResultsData->distance_mm[ZoneNum] > 0) && ((ResultsData->target_status[ZoneNum] == 5) || (ResultsData->target_status[ZoneNum] == 6) || (ResultsData->target_status[ZoneNum] == 9)) )
		{
			Hyp = ResultsData->distance_mm[ZoneNum]/SinOfPitch[ZoneNum];
			XYZ_ZoneCoordinates->Xpos[ZoneNum] = CosOfYaw[ZoneNum]*CosOfPitch[ZoneNum]*Hyp;
			XYZ_ZoneCoordinates->Ypos[ZoneNum] = SinOfYaw[ZoneNum]*CosOfPitch[ZoneNum]*Hyp;
			XYZ_ZoneCoordinates->Zpos[ZoneNum] = ResultsData->distance_mm[ZoneNum];
		}
		else
		{
			XYZ_ZoneCoordinates->Xpos[ZoneNum] = 0;
			XYZ_ZoneCoordinates->Ypos[ZoneNum] = 0;
			XYZ_ZoneCoordinates->Zpos[ZoneNum] = 0;
		}
	}
	return 0;
}
 
uint8_t PrintXYZCoords(XYZ_ZoneCoordinates_t *XYZ_ZoneCoordinates)
{
	uint8_t i, j;
	printf("XYZ Coordinates for the target in each zone\n");
	for (i = 0; i < 8; i++) \
	{
		for (j = 0; j < 8; j++)
		{
			idx = (i * 8 + j);
			printf("%5.0f, %5.0f, %5.0f |",XYZ_ZoneCoordinates->Xpos[idx],XYZ_ZoneCoordinates->Ypos[idx],XYZ_ZoneCoordinates->Zpos[idx]);
		}
		printf("\n");
	}
	printf("\n");
 
	return 0;
}

void app_main(void)
{

    //Define the i2c bus configuration
    i2c_port_t i2c_port = I2C_NUM_0;
    i2c_master_bus_config_t i2c_mst_config = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = i2c_port,
            .scl_io_num = GPIO_NUM_23,
            .sda_io_num = GPIO_NUM_24,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = false,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    //Define the i2c device configuration
    i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            // .device_address = VL53L5CX_DEFAULT_I2C_ADDRESS >> 1,
            .device_address = 0x29,
            .scl_speed_hz = VL53L5CX_MAX_CLK_SPEED,
    };

    /*********************************/
    /*   VL53L5CX ranging variables  */
    /*********************************/

    uint8_t 				status, isAlive, isReady;
    VL53L5CX_Configuration 	Dev;			/* Sensor configuration */
    VL53L5CX_ResultsData 	Results;		/* Results data from VL53L5CX */


    /*********************************/
    /*      Customer platform        */
    /*********************************/

    Dev.platform.bus_config = i2c_mst_config;

    //Register the device
    i2c_master_bus_add_device(bus_handle, &dev_cfg, &Dev.platform.handle);

    /* (Optional) Reset sensor */
    Dev.platform.reset_gpio = GPIO_NUM_28;
    VL53L5CX_Reset_Sensor(&(Dev.platform));

    /* (Optional) Set a new I2C address if the wanted address is different
    * from the default one (filled with 0x20 for this example).
    */
    //status = vl53l5cx_set_i2c_address(&Dev, 0x20);


    /*********************************/
    /*   Power on sensor and init    */
    /*********************************/

    /* (Optional) Check if there is a VL53L5CX sensor connected */
    status = vl53l5cx_is_alive(&Dev, &isAlive);
    if(!isAlive || status)
    {
        printf("VL53L5CX not detected at requested address\n");
        return;
    }

    /* (Mandatory) Init VL53L5CX sensor */
    status = vl53l5cx_init(&Dev);
    if(status)
    {
        printf("VL53L5CX ULD Loading failed\n");
        return;
    }

    printf("VL53L5CX ULD ready ! (Version : %s)\n",
           VL53L5CX_API_REVISION);


    /*********************************/
    /*         Ranging loop          */
    /*********************************/
    ComputeSinCosTables();
    vl53l5cx_set_resolution(&Dev, VL53L5CX_RESOLUTION_8X8); // Set resolution to 4x4

    status = vl53l5cx_start_ranging(&Dev);

    while(1)
    {
        /* Use polling function to know when a new measurement is ready.
         * Another way can be to wait for HW interrupt raised on PIN A1
         * (INT) when a new measurement is ready */

        status = vl53l5cx_check_data_ready(&Dev, &isReady);

        if(isReady)
        {
            vl53l5cx_get_ranging_data(&Dev, &Results);

            /* As the sensor is set in 4x4 mode by default, we have a total
             * of 16 zones to print. For this example, only the data of first zone are
             * print */
            printf("Print data no : %3u\n", Dev.streamcount);
            ConvertDist2XYZCoords8x8(&Results, &XYZ_ZoneCoordinates);
            PrintXYZCoords(&XYZ_ZoneCoordinates);
        }
        /* Wait a few ms to avoid too high polling (function in platform
         * file, not in API) */
        VL53L5CX_WaitMs(&(Dev.platform), 5);
    }

    status = vl53l5cx_stop_ranging(&Dev);
    printf("End of ULD demo\n");
}