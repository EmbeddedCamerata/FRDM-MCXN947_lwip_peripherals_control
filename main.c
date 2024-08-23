/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020,2022-2023  NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "lwip/opt.h"
#include "lwip/sockets.h"
#include "fsl_adapter_gpio.h"
#include "fsl_debug_console.h"

#include "lwip/netifapi.h"
#include "lwip/tcpip.h"
#include "netif/ethernet.h"
#include "ethernetif.h"

#include "pin_mux.h"
#include "board.h"
#ifndef configMAC_ADDR
#include "fsl_silicon_id.h"
#endif
#include "fsl_phy.h"

#include "fsl_enet.h"
#include "fsl_phylan8741.h"

#include "fsl_lpadc.h"

#include "fsl_common.h"
#include "fsl_vref.h"
#include "fsl_spc.h"

#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"

#include "fsl_tsi_v6.h"
#include "fsl_lptmr.h"

#if defined(FSL_FEATURE_SOC_PORT_COUNT) && (FSL_FEATURE_SOC_PORT_COUNT)
#include "fsl_port.h"
#endif
#include "fsl_gpio.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* IP address configuration. */
#ifndef configIP_ADDR0
#define configIP_ADDR0 192
#endif
#ifndef configIP_ADDR1
#define configIP_ADDR1 168
#endif
#ifndef configIP_ADDR2
#define configIP_ADDR2 18
#endif
#ifndef configIP_ADDR3
#define configIP_ADDR3 100
#endif

/* Netmask configuration. */
#ifndef configNET_MASK0
#define configNET_MASK0 255
#endif
#ifndef configNET_MASK1
#define configNET_MASK1 255
#endif
#ifndef configNET_MASK2
#define configNET_MASK2 255
#endif
#ifndef configNET_MASK3
#define configNET_MASK3 0
#endif

/* Gateway address configuration. */
#ifndef configGW_ADDR0
#define configGW_ADDR0 192
#endif
#ifndef configGW_ADDR1
#define configGW_ADDR1 168
#endif
#ifndef configGW_ADDR2
#define configGW_ADDR2 18
#endif
#ifndef configGW_ADDR3
#define configGW_ADDR3 99
#endif

#define TCP_PORT 5001
#define LWIP_SEND_DATA_SIZE 128
#define LWIP_RECV_DATA_SIZE 128

/* Ethernet configuration. */
extern phy_lan8741_resource_t g_phy_resource;
#define ENET_BASE ENET0
#define PHY_ADDRESS BOARD_ENET0_PHY_ADDRESS
#define PHY_OPS &phylan8741_ops
#define PHY_RESOURCE &g_phy_resource
#define CLOCK_FREQ (50000000U)

#ifndef NETIF_INIT_FN
/*! @brief Network interface initialization function. */
#define NETIF_INIT_FN ethernetif0_init
#endif /* NETIF_INIT_FN */

/*! @brief Stack size of the temporary lwIP initialization thread. */
#define INIT_THREAD_STACKSIZE 1024

/*! @brief Priority of the temporary lwIP initialization thread. */
#define INIT_THREAD_PRIO DEFAULT_THREAD_PRIO

/*! @brief Selection of GPIO perihperal and its pin for the reception of PHY interrupts. */
#if ETH_LINK_POLLING_INTERVAL_MS == 0
#ifndef PHY_INT_PORT
#if (!defined(BOARD_NETWORK_USE_100M_ENET_PORT) || !BOARD_NETWORK_USE_100M_ENET_PORT) && \
    defined(BOARD_INITENET1GPINS_PHY_INTR_PERIPHERAL)
#define PHY_INT_PORT BOARD_INITENET1GPINS_PHY_INTR_PERIPHERAL
#elif defined(BOARD_INITENETPINS_PHY_INTR_PERIPHERAL)
#define PHY_INT_PORT BOARD_INITENETPINS_PHY_INTR_PERIPHERAL
#elif defined(BOARD_INITPINS_PHY_INTR_PERIPHERAL)
#define PHY_INT_PORT BOARD_INITPINS_PHY_INTR_PERIPHERAL
#else
#error "Interrupt-based link-state detection was enabled on an unsupported board."
#endif
#endif // #ifndef PHY_INT_PORT

#ifndef PHY_INT_PIN
#if (!defined(BOARD_NETWORK_USE_100M_ENET_PORT) || !BOARD_NETWORK_USE_100M_ENET_PORT) && \
    defined(BOARD_INITENET1GPINS_PHY_INTR_CHANNEL)
#define PHY_INT_PIN BOARD_INITENET1GPINS_PHY_INTR_CHANNEL
#elif defined(BOARD_INITENETPINS_PHY_INTR_CHANNEL)
#define PHY_INT_PIN BOARD_INITENETPINS_PHY_INTR_CHANNEL
#elif defined(BOARD_INITPINS_PHY_INTR_CHANNEL)
#define PHY_INT_PIN BOARD_INITPINS_PHY_INTR_CHANNEL
#else
#error "Interrupt-based link-state detection was enabled on an unsupported board."
#endif
#endif // #ifndef PHY_INT_PIN
#endif // #if ETH_LINK_POLLING_INTERVAL_MS == 0

#define SPC_BASE SPC0
#define LPADC_BASE ADC0
#define LPADC_IRQn ADC0_IRQn
#define LPADC_IRQ_HANDLER_FUNC ADC0_IRQHandler
#define LPADC_TEMP_SENS_CHANNEL 26U
#define LPADC_USER_CMDID 1U /* CMD1 */
#define LPADC_SAMPLE_CHANNEL_MODE kLPADC_SampleChannelDiffBothSide
/* Use VREF_OUT driven from the VREF block as the reference volatage,
   note that the bit combinations for controlling the LPADC reference voltage
   on different chips are different, see chip Reference Manual for details. */
#define LPADC_VREF_SOURCE kLPADC_ReferenceVoltageAlt2
#define LPADC_DO_OFFSET_CALIBRATION false
#define LPADC_OFFSET_VALUE_A 0xAU
#define LPADC_OFFSET_VALUE_B 0xAU
#define LPADC_USE_HIGH_RESOLUTION true
#define LPADC_TEMP_PARAMETER_A FSL_FEATURE_LPADC_TEMP_PARAMETER_A
#define LPADC_TEMP_PARAMETER_B FSL_FEATURE_LPADC_TEMP_PARAMETER_B
#define LPADC_TEMP_PARAMETER_ALPHA FSL_FEATURE_LPADC_TEMP_PARAMETER_ALPHA
#define VREF_BASE VREF0

/* Available PAD names on board */
#define PAD_TSI_ELECTRODE_1_NAME "E1"

/* IRQ related redefinitions for specific SOC */
#define TSI0_IRQHandler TSI_END_OF_SCAN_IRQHandler
#define TSI0_IRQn TSI_END_OF_SCAN_IRQn

/* Define the delta value to indicate a touch event */
#define TOUCH_DELTA_VALUE 100U

/* Get source clock for LPTMR driver */
#define LPTMR_SOURCE_CLOCK (16000)
/* Define LPTMR microseconds count value */
#define LPTMR_USEC_COUNT (100000)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void lwip_tcp_task(void *arg);
static void tsi_scan_task(void *arg);
static void sw_scan_task(void *arg);
static void send_data_task(void *arg);
static void ADC_Configuration(void);
static void temp_measurement_task(void *arg);
/*******************************************************************************
 * Variables
 ******************************************************************************/
phy_lan8741_resource_t g_phy_resource;

static phy_handle_t phyHandle;
static struct netif netif;

TaskHandle_t send_data_task_handle = NULL;

lpadc_conv_command_config_t g_LpadcCommandConfigStruct; /* Structure to configure conversion command. */
volatile bool g_LpadcConversionCompletedFlag = false;
float g_CurrentTemperature = 0.0f;

tsi_calibration_data_t buffer;
static volatile bool s_tsiInProgress = true;
/* Array of TSI peripheral base address. */
#if defined(TSI0)
#define APP_TSI TSI0
#elif defined(TSI)
#define APP_TSI TSI
#endif

/* Whether the SW button is pressed */
volatile uint32_t g_ButtonStatus = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/
void TSI0_IRQHandler(void)
{
#if BOARD_TSI_ELECTRODE_1 > 15
    /* errata ERR051410: When reading TSI_COMFIG[TSICH] bitfield, the upper most bit will always be 0. */
    if ((TSI_GetSelfCapMeasuredChannel(APP_TSI) | 0x10U) == BOARD_TSI_ELECTRODE_1)
#else
    if (TSI_GetSelfCapMeasuredChannel(APP_TSI) == BOARD_TSI_ELECTRODE_1)
#endif
    {
        if (TSI_GetCounter(APP_TSI) > (uint16_t)(buffer.calibratedData[BOARD_TSI_ELECTRODE_1] + TOUCH_DELTA_VALUE))
        {
            // LED_BLUE_TOGGLE(); /* Toggle the touch event indicating LED */
            s_tsiInProgress = false;
        }
    }

    /* Clear endOfScan flag */
    TSI_ClearStatusFlags(APP_TSI, kTSI_EndOfScanFlag);
    SDK_ISR_EXIT_BARRIER;
}

static void MDIO_Init(void)
{
    (void)CLOCK_EnableClock(s_enetClock[ENET_GetInstance(ENET_BASE)]);
    ENET_SetSMI(ENET_BASE, CLOCK_GetCoreSysClkFreq());
}

static status_t MDIO_Write(uint8_t phyAddr, uint8_t regAddr, uint16_t data)
{
    return ENET_MDIOWrite(ENET_BASE, phyAddr, regAddr, data);
}

static status_t MDIO_Read(uint8_t phyAddr, uint8_t regAddr, uint16_t *pData)
{
    return ENET_MDIORead(ENET_BASE, phyAddr, regAddr, pData);
}

/*!
 * @brief Initializes lwIP stack.
 */
static void stack_init(void *arg)
{
    ip4_addr_t netif_ipaddr, netif_netmask, netif_gw;
    ethernetif_config_t enet_config = {
        .phyHandle = &phyHandle,
        .phyAddr = PHY_ADDRESS,
        .phyOps = PHY_OPS,
        .phyResource = PHY_RESOURCE,
        .srcClockHz = CLOCK_FREQ,
#ifdef configMAC_ADDR
        .macAddress = configMAC_ADDR,
#endif
#if ETH_LINK_POLLING_INTERVAL_MS == 0
        .phyIntGpio = PHY_INT_PORT,
        .phyIntGpioPin = PHY_INT_PIN
#endif
    };

    LWIP_UNUSED_ARG(arg);

    /* Set MAC address. */
#ifndef configMAC_ADDR
    (void)SILICONID_ConvertToMacAddr(&enet_config.macAddress);
#endif

    IP4_ADDR(&netif_ipaddr, configIP_ADDR0, configIP_ADDR1, configIP_ADDR2, configIP_ADDR3);
    IP4_ADDR(&netif_netmask, configNET_MASK0, configNET_MASK1, configNET_MASK2, configNET_MASK3);
    IP4_ADDR(&netif_gw, configGW_ADDR0, configGW_ADDR1, configGW_ADDR2, configGW_ADDR3);

    tcpip_init(NULL, NULL);

    HAL_GpioPreInit();

    netifapi_netif_add(&netif, &netif_ipaddr, &netif_netmask, &netif_gw, &enet_config, NETIF_INIT_FN,
                       tcpip_input);
    netifapi_netif_set_default(&netif);
    netifapi_netif_set_up(&netif);

    while (ethernetif_wait_linkup(&netif, 5000) != ERR_OK)
    {
        PRINTF("PHY Auto-negotiation failed. Please check the cable connection and link partner setting.\n");
    }

    PRINTF("\n************************************************\n");
    PRINTF(" IPv4 Address     : %u.%u.%u.%u\n", ((u8_t *)&netif_ipaddr)[0], ((u8_t *)&netif_ipaddr)[1],
           ((u8_t *)&netif_ipaddr)[2], ((u8_t *)&netif_ipaddr)[3]);
    PRINTF(" IPv4 Subnet mask : %u.%u.%u.%u\n", ((u8_t *)&netif_netmask)[0], ((u8_t *)&netif_netmask)[1],
           ((u8_t *)&netif_netmask)[2], ((u8_t *)&netif_netmask)[3]);
    PRINTF(" IPv4 Gateway     : %u.%u.%u.%u\n", ((u8_t *)&netif_gw)[0], ((u8_t *)&netif_gw)[1],
           ((u8_t *)&netif_gw)[2], ((u8_t *)&netif_gw)[3]);
    PRINTF("************************************************\n");

    if (sys_thread_new("lwip_tcp_task", lwip_tcp_task, NULL, INIT_THREAD_STACKSIZE, INIT_THREAD_PRIO) == NULL)
        LWIP_ASSERT("stack_init(): Task creation failed.", 0);

    vTaskDelete(NULL);
}

static void lwip_tcp_task(void *arg)
{
    int err, sock;
    struct timeval timeout = {.tv_sec = 1, .tv_usec = 0};
    struct sockaddr_in server_addr, client_addr;
    int connected = -1;
    socklen_t sin_size = sizeof(struct sockaddr_in);
    char *recv_cmd;
    ssize_t recv_cmd_len;
    char unknown_cmd_str[] = "Unknown command\n";

    LWIP_UNUSED_ARG(arg);

    recv_cmd = (char *)pvPortMalloc(LWIP_RECV_DATA_SIZE);
    LWIP_ERROR("no memory.", recv_cmd != NULL, goto __exit;);

    sock = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    LWIP_ERROR("socket build failed.", sock >= 0, return;);

    err = lwip_setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    LWIP_ERROR("setting receive timeout failed.", err == 0, goto __exit;);

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = lwip_htons(TCP_PORT);
    memset(&(server_addr.sin_zero), 0, sizeof(server_addr.sin_zero));

    /* Socket bind */
    err = lwip_bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
    LWIP_ERROR("bind failed.", err == 0, goto __exit;);

    /* Listen */
    err = lwip_listen(sock, 5);
    LWIP_ERROR("listen failed.", err == 0, goto __exit;);

    while (1)
    {
        connected = lwip_accept(sock, (struct sockaddr *)&client_addr, &sin_size);

        if (connected > 0)
        {
            PRINTF("new client connected from (%s, %d)\n",
                   inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

            if (send_data_task_handle == NULL)
            {
                if (xTaskCreate(send_data_task, "send_data_task", 512,
                                (void *)&connected, 4, &send_data_task_handle) != pdPASS)
                {
                    PRINTF("lwip_tcp_task(): Task creation failed!\n");
                    while (1)
                        ;
                }
            }

            while (1)
            {
                memset(recv_cmd, 0, LWIP_RECV_DATA_SIZE);
                recv_cmd_len = lwip_recv(connected, recv_cmd, LWIP_RECV_DATA_SIZE, 0);

                if (recv_cmd_len <= 0)
                    break;

                /* Command match */
                if (strcmp(recv_cmd, "close") == 0)
                {
                    err = lwip_close(connected);
                    LWIP_ERROR("close failed", err == 0, goto __exit;);
                    connected = -1;
                    break;
                }
                else if (strcmp(recv_cmd, "blue") == 0)
                {
                    LED_BLUE_TOGGLE();
                }
                else if (strcmp(recv_cmd, "blue off") == 0)
                {
                    LED_BLUE_OFF();
                }
                else if (strcmp(recv_cmd, "blue on") == 0)
                {
                    LED_BLUE_ON();
                }
                else if (strcmp(recv_cmd, "green") == 0)
                {
                    LED_GREEN_TOGGLE();
                }
                else if (strcmp(recv_cmd, "green off") == 0)
                {
                    LED_GREEN_OFF();
                }
                else if (strcmp(recv_cmd, "green on") == 0)
                {
                    LED_GREEN_ON();
                }
                else if (strcmp(recv_cmd, "red") == 0)
                {
                    LED_RED_TOGGLE();
                }
                else if (strcmp(recv_cmd, "red off") == 0)
                {
                    LED_RED_OFF();
                }
                else if (strcmp(recv_cmd, "red on") == 0)
                {
                    LED_RED_ON();
                }
                else // Send unknown command message
                {
                    lwip_write(connected, (void *)unknown_cmd_str, strlen(unknown_cmd_str));
                }
            }
        }
        else
        {
            if (send_data_task_handle != NULL)
            {
                vTaskDelete(send_data_task_handle);
                send_data_task_handle = NULL;
            }
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

__exit:
    if (recv_cmd)
        vPortFree(recv_cmd);
    if (sock >= 0)
        lwip_close(sock);

    vTaskDelete(NULL);
}

static void send_data_task(void *arg)
{
    int connected = *(int *)arg;
    int temp_int, temp_deci;
    char *send_data;
    send_data = (char *)pvPortMalloc(LWIP_SEND_DATA_SIZE);
    LWIP_ERROR("no memory", send_data != NULL, goto __exit;);

    LWIP_UNUSED_ARG(arg);

    while (1)
    {
        temp_int = (int)g_CurrentTemperature;
        temp_deci = (int)(100 * (g_CurrentTemperature - temp_int));
        sprintf(send_data, "temp: %02d.%02d, touch btn: %d, sw3: %lu\n",
                temp_int, temp_deci, 1 - s_tsiInProgress, 1 - g_ButtonStatus);

        lwip_write(connected, (void *)send_data, strlen(send_data));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);

__exit:
    if (send_data)
        vPortFree(send_data);

    vTaskDelete(NULL);
}

static void tsi_scan_task(void *arg)
{
    while (1)
    {
        while (s_tsiInProgress)
        {
            TSI_StartSoftwareTrigger(APP_TSI);
        }
        s_tsiInProgress = true;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

static void temp_measurement_task(void *arg)
{
    while (1)
    {
        g_LpadcConversionCompletedFlag = false;
        LPADC_DoSoftwareTrigger(LPADC_BASE, 1U); /* 1U is trigger0 mask. */
        while (false == g_LpadcConversionCompletedFlag)
            ;
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

static void sw_scan_task(void *arg)
{
    while (1)
    {
        if ((g_ButtonStatus = GPIO_PinRead(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN)) == 0)
        {
            // LED_RED_TOGGLE();
            PRINTF("%s is pressed\n", BOARD_SW3_NAME);
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

float MeasureTemperature(ADC_Type *base, uint32_t commandId, uint32_t index)
{
    lpadc_conv_result_t convResultStruct;
    uint16_t Vbe1 = 0U;
    uint16_t Vbe8 = 0U;
    uint32_t convResultShift = 3U;
    float parameterSlope = LPADC_TEMP_PARAMETER_A;
    float parameterOffset = LPADC_TEMP_PARAMETER_B;
    float parameterAlpha = LPADC_TEMP_PARAMETER_ALPHA;
    float temperature = -273.15f; /* Absolute zero degree as the incorrect return value. */

#if defined(FSL_FEATURE_LPADC_TEMP_SENS_BUFFER_SIZE) && (FSL_FEATURE_LPADC_TEMP_SENS_BUFFER_SIZE == 4U)
    /* For best temperature measure performance, the recommended LOOP Count should be 4, but the first two results is
     * useless. */
    /* Drop the useless result. */
    (void)LPADC_GetConvResult(base, &convResultStruct, (uint8_t)index);
    (void)LPADC_GetConvResult(base, &convResultStruct, (uint8_t)index);
#endif /* FSL_FEATURE_LPADC_TEMP_SENS_BUFFER_SIZE */

    /* Read the 2 temperature sensor result. */
    if (true == LPADC_GetConvResult(base, &convResultStruct, (uint8_t)index))
    {
        Vbe1 = convResultStruct.convValue >> convResultShift;
        if (true == LPADC_GetConvResult(base, &convResultStruct, (uint8_t)index))
        {
            Vbe8 = convResultStruct.convValue >> convResultShift;
            /* Final temperature = A*[alpha*(Vbe8-Vbe1)/(Vbe8 + alpha*(Vbe8-Vbe1))] - B. */
            temperature = parameterSlope * (parameterAlpha * ((float)Vbe8 - (float)Vbe1) /
                                            ((float)Vbe8 + parameterAlpha * ((float)Vbe8 - (float)Vbe1))) -
                          parameterOffset;
        }
    }

    return temperature;
}

int main(void)
{
    vref_config_t vrefConfig;
    tsi_selfCap_config_t tsiConfig_selfCap;
    lptmr_config_t lptmrConfig;
    memset((void *)&lptmrConfig, 0, sizeof(lptmrConfig));

    /* attach FRO 12M to FLEXCOMM4 (debug console) */
    CLOCK_SetClkDiv(kCLOCK_DivFlexcom4Clk, 1u);
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* attach FRO HF to ADC0 */
    CLOCK_SetClkDiv(kCLOCK_DivAdc0Clk, 1U);
    CLOCK_AttachClk(kFRO_HF_to_ADC0);

    /* Enables the clock for INPUTMUX: Enables clock */
    CLOCK_EnableClock(kCLOCK_InputMux);
    /* Enables the clk_16k[1] */
    CLOCK_SetupClk16KClocking(kCLOCK_Clk16KToVsys);
    /* attach FRO HF to SCT */
    CLOCK_SetClkDiv(kCLOCK_DivTsiClk, 1u);
    CLOCK_AttachClk(kCLK_IN_to_TSI);

    /* enable VREF */
    SPC_EnableActiveModeAnalogModules(SPC_BASE, kSPC_controlVref);

    VREF_GetDefaultConfig(&vrefConfig);
    /* Initialize VREF module, the VREF module provides reference voltage and bias current for LPADC. */
    VREF_Init(VREF_BASE, &vrefConfig);
    /* Get a 1.8V reference voltage. */
    VREF_SetTrim21Val(VREF_BASE, 8U);

    BOARD_InitPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    /* Define the init structure for the input switch pin */
    gpio_pin_config_t sw_config = {
        kGPIO_DigitalInput,
        0,
    };
    GPIO_PinInit(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN, &sw_config);

    /* LED init */
    LED_RED_INIT(LOGIC_LED_OFF);
    LED_GREEN_INIT(LOGIC_LED_OFF);
    LED_BLUE_INIT(LOGIC_LED_OFF);

    ADC_Configuration();

    CLOCK_AttachClk(MUX_A(CM_ENETRMIICLKSEL, 0));
    CLOCK_EnableClock(kCLOCK_Enet);
    SYSCON0->PRESETCTRL2 = SYSCON_PRESETCTRL2_ENET_RST_MASK;
    SYSCON0->PRESETCTRL2 &= ~SYSCON_PRESETCTRL2_ENET_RST_MASK;

    MDIO_Init();

    g_phy_resource.read = MDIO_Read;
    g_phy_resource.write = MDIO_Write;

    /* Configure LPTMR */
    LPTMR_GetDefaultConfig(&lptmrConfig);
    /* TSI default hardware configuration for self-cap mode */
    TSI_GetSelfCapModeDefaultConfig(&tsiConfig_selfCap);

    /* Initialize the LPTMR */
    LPTMR_Init(LPTMR0, &lptmrConfig);
    /* Initialize the TSI */
    TSI_InitSelfCapMode(APP_TSI, &tsiConfig_selfCap);
    /* Enable noise cancellation function */
    TSI_EnableNoiseCancellation(APP_TSI, true);

    /* Set timer period */
    LPTMR_SetTimerPeriod(LPTMR0, USEC_TO_COUNT(LPTMR_USEC_COUNT, LPTMR_SOURCE_CLOCK));

    NVIC_EnableIRQ(TSI0_IRQn);
    TSI_EnableModule(APP_TSI, true); /* Enable module */

    /* Self-cap calibrate */
    memset((void *)&buffer, 0, sizeof(buffer));
    TSI_SelfCapCalibrate(APP_TSI, &buffer);

    /* Software trigger scan using interrupt method */
    TSI_EnableInterrupts(APP_TSI, kTSI_GlobalInterruptEnable);
    TSI_EnableInterrupts(APP_TSI, kTSI_EndOfScanInterruptEnable);
    TSI_ClearStatusFlags(APP_TSI, kTSI_EndOfScanFlag);
    TSI_SetSelfCapMeasuredChannel(APP_TSI, BOARD_TSI_ELECTRODE_1);

    if (xTaskCreate(tsi_scan_task, "tsi_scan_task", 512, NULL, 3, NULL) != pdPASS)
    {
        PRINTF("main(): Task creation failed!\n");
        while (1)
            ;
    }

    if (xTaskCreate(temp_measurement_task, "temp_measurement_task", 512, NULL, 4, NULL) != pdPASS)
    {
        PRINTF("main(): Task creation failed!\n");
        while (1)
            ;
    }

    if (xTaskCreate(sw_scan_task, "sw_scan_task", 512, NULL, 4, NULL) != pdPASS)
    {
        PRINTF("main(): Task creation failed!\n");
        while (1)
            ;
    }

    /* Initialize lwIP from thread */
    if (sys_thread_new("main", stack_init, NULL, INIT_THREAD_STACKSIZE, INIT_THREAD_PRIO) == NULL)
        LWIP_ASSERT("main(): Task creation failed.", 0);

    vTaskStartScheduler();

    /* Will not get here unless a task calls vTaskEndScheduler() */
    return 0;
}

void LPADC_IRQ_HANDLER_FUNC(void)
{
    g_CurrentTemperature = MeasureTemperature(LPADC_BASE, LPADC_USER_CMDID, 0U);
    g_LpadcConversionCompletedFlag = true;
    SDK_ISR_EXIT_BARRIER;
}

static void ADC_Configuration(void)
{
    lpadc_config_t lpadcConfigStruct;
    lpadc_conv_trigger_config_t lpadcTriggerConfigStruct;

    /* Init ADC peripheral. */
    LPADC_GetDefaultConfig(&lpadcConfigStruct);
    lpadcConfigStruct.enableAnalogPreliminary = true;
    lpadcConfigStruct.powerLevelMode = kLPADC_PowerLevelAlt4;
#if defined(LPADC_VREF_SOURCE)
    lpadcConfigStruct.referenceVoltageSource = LPADC_VREF_SOURCE;
#endif /* LPADC_VREF_SOURCE */
#if defined(FSL_FEATURE_LPADC_HAS_CTRL_CAL_AVGS) && FSL_FEATURE_LPADC_HAS_CTRL_CAL_AVGS
    lpadcConfigStruct.conversionAverageMode = kLPADC_ConversionAverage128;
#endif /* FSL_FEATURE_LPADC_HAS_CTRL_CAL_AVGS */
#if defined(FSL_FEATURE_LPADC_TEMP_SENS_BUFFER_SIZE)
    lpadcConfigStruct.FIFO0Watermark = FSL_FEATURE_LPADC_TEMP_SENS_BUFFER_SIZE - 1U;
#endif /* FSL_FEATURE_LPADC_TEMP_SENS_BUFFER_SIZE */
    LPADC_Init(LPADC_BASE, &lpadcConfigStruct);
#if (defined(FSL_FEATURE_LPADC_FIFO_COUNT) && (FSL_FEATURE_LPADC_FIFO_COUNT == 2U))
    LPADC_DoResetFIFO0(LPADC_BASE);
#else
    LPADC_DoResetFIFO(LPADC_BASE);
#endif

    /* Do ADC calibration. */
#if defined(FSL_FEATURE_LPADC_HAS_CTRL_CALOFS) && FSL_FEATURE_LPADC_HAS_CTRL_CALOFS
#if defined(FSL_FEATURE_LPADC_HAS_OFSTRIM) && FSL_FEATURE_LPADC_HAS_OFSTRIM
    /* Request offset calibration. */
#if defined(LPADC_DO_OFFSET_CALIBRATION) && LPADC_DO_OFFSET_CALIBRATION
    LPADC_DoOffsetCalibration(LPADC_BASE);
#else
    LPADC_SetOffsetValue(LPADC_BASE, LPADC_OFFSET_VALUE_A, LPADC_OFFSET_VALUE_B);
#endif /* LPADC_DO_OFFSET_CALIBRATION */
#endif /* FSL_FEATURE_LPADC_HAS_OFSTRIM */
    /* Request gain calibration. */
    LPADC_DoAutoCalibration(LPADC_BASE);
#endif /* FSL_FEATURE_LPADC_HAS_CTRL_CALOFS */

    /* Set conversion CMD configuration. */
    LPADC_GetDefaultConvCommandConfig(&g_LpadcCommandConfigStruct);
    g_LpadcCommandConfigStruct.channelNumber = LPADC_TEMP_SENS_CHANNEL;
    g_LpadcCommandConfigStruct.sampleChannelMode = LPADC_SAMPLE_CHANNEL_MODE;
    g_LpadcCommandConfigStruct.sampleTimeMode = kLPADC_SampleTimeADCK131;
    g_LpadcCommandConfigStruct.hardwareAverageMode = kLPADC_HardwareAverageCount128;
#if defined(FSL_FEATURE_LPADC_TEMP_SENS_BUFFER_SIZE)
    g_LpadcCommandConfigStruct.loopCount = FSL_FEATURE_LPADC_TEMP_SENS_BUFFER_SIZE - 1U;
#endif /* FSL_FEATURE_LPADC_TEMP_SENS_BUFFER_SIZE */
#if defined(FSL_FEATURE_LPADC_HAS_CMDL_MODE) && FSL_FEATURE_LPADC_HAS_CMDL_MODE
    g_LpadcCommandConfigStruct.conversionResolutionMode = kLPADC_ConversionResolutionHigh;
#endif /* FSL_FEATURE_LPADC_HAS_CMDL_MODE */
    LPADC_SetConvCommandConfig(LPADC_BASE, LPADC_USER_CMDID, &g_LpadcCommandConfigStruct);

    /* Set trigger configuration. */
    LPADC_GetDefaultConvTriggerConfig(&lpadcTriggerConfigStruct);
    lpadcTriggerConfigStruct.targetCommandId = LPADC_USER_CMDID;
    LPADC_SetConvTriggerConfig(LPADC_BASE, 0U, &lpadcTriggerConfigStruct); /* Configurate the trigger0. */

    /* Enable the watermark interrupt. */
#if (defined(FSL_FEATURE_LPADC_FIFO_COUNT) && (FSL_FEATURE_LPADC_FIFO_COUNT == 2U))
    LPADC_EnableInterrupts(LPADC_BASE, kLPADC_FIFO0WatermarkInterruptEnable);
#else
    LPADC_EnableInterrupts(LPADC_BASE, kLPADC_FIFOWatermarkInterruptEnable);
#endif /* FSL_FEATURE_LPADC_FIFO_COUNT */
    EnableIRQ(LPADC_IRQn);

    /* Eliminate the first two inaccurate results. */
    LPADC_DoSoftwareTrigger(LPADC_BASE, 1U); /* 1U is trigger0 mask. */
    while (false == g_LpadcConversionCompletedFlag)
    {
    }
}
