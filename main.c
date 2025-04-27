/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for Hello World Example using HAL APIs.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2022-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"



int main(void)
{
    cy_rslt_t result;
    int16_t adcResult0,adcResult1;
    uint32_t count = 0;


#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif /* #if defined (CY_DEVICE_SECURE) */

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init_fc(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
            CYBSP_DEBUG_UART_CTS,CYBSP_DEBUG_UART_RTS,CY_RETARGET_IO_BAUDRATE);

    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
   // Cy_SysAnalog_Init(&pass_0_aref_0_config);
	Cy_SysAnalog_Init(&pass_0_aref_0_config);
	// Initialize AREF
	Cy_SysAnalog_Enable();
    Cy_GPIO_Pin_Init(CYBSP_A0_PORT, CYBSP_A0_PIN, &CYBSP_A0_config);
    Cy_GPIO_Pin_Init(CYBSP_A3_PORT, CYBSP_A3_PIN, &CYBSP_A3_config);
    Cy_GPIO_Pin_Init(CYBSP_LED8_PORT, CYBSP_LED8_PIN, &CYBSP_LED8_config);

	/* Initialize the SAR module */
	Cy_SAR_Init(SAR, &pass_0_sar_0_config);
	/* Enable the SAR module */
	Cy_SAR_Enable(SAR);

    if (CY_TCPWM_SUCCESS != Cy_TCPWM_PWM_Init(TCPWM1, tcpwm_1_cnt_14_NUM, &tcpwm_1_cnt_14_config))
       {
    	CY_ASSERT(0);
       }
	Cy_TCPWM_PWM_Enable(TCPWM1, tcpwm_1_cnt_14_NUM);

	Cy_TCPWM_TriggerStart_Single(TCPWM1, tcpwm_1_cnt_14_NUM);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** "
           "ADC read data example "
           "****************** \r\n\n");


    for (;;)
    {
    	 /* Start the continuous conversion */
    	        Cy_SAR_StartConvert(SAR, CY_SAR_START_CONVERT_CONTINUOUS );
    	        /* Wait till the sample is ready */
    	        Cy_SAR_IsEndConversion(SAR, CY_SAR_WAIT_FOR_RESULT);
    	        /* Get the result from Input 0 */
    	        adcResult0 =0X7ff & Cy_SAR_GetResult16(SAR, 0);
    	       // adcResult1 =0X7ff & Cy_SAR_GetResult16(SAR, 3);

    	        /* Map ADC result to PWM duty cycle */
    	       float duty_cycle = ((float)adcResult0  / 2048.0f
    	    		   ) * 32767; // Assuming 12-bit ADC
    	        Cy_TCPWM_PWM_SetCompare0(TCPWM1, tcpwm_1_cnt_14_NUM,(uint16_t)duty_cycle);
    	        printf("Count: %lu, ADC Result Channel 0 = %d mV, Duty Cycle = %f\r\n",adcResult0, Cy_SAR_CountsTo_mVolts(SAR, 0, adcResult0),duty_cycle);
    	        count++;
    	        Cy_SysLib_Delay(1000);

    }
}



/* [] END OF FILE */
