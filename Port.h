/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Ammar Moataz
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

/* Id for the company in the AUTOSAR */
#define PORT_VENDOR_ID (1000U)

/* Port Module Id */
#define PORT_MODULE_ID (120U)

/* Port Instance Id */
#define PORT_INSTANCE_ID (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION (1U)
#define PORT_SW_MINOR_VERSION (0U)
#define PORT_SW_PATCH_VERSION (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION (4U)
#define PORT_AR_RELEASE_MINOR_VERSION (0U)
#define PORT_AR_RELEASE_PATCH_VERSION (3U)

/*
 * Macros for Port Status
 */
#define PORT_INITIALIZED (1U)
#define PORT_NOT_INITIALIZED (0U)

/* Standard AUTOSAR types */
#include "Std_Types.h"

/* AUTOSAR checking between Std Types and Port   Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION) || (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION) || (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Std_Types.h does not match the expected version"
#endif

/* Port Pre-Compile Configuration Header file */
#include "Port_Cfg.h"

/* AUTOSAR Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION) || (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION) || (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Port_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION) || (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION) || (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
#error "The SW version of Port_Cfg.h does not match the expected version"
#endif

/* Non AUTOSAR files */
#include "Common_Macros.h"

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/
/* Service ID for Port Init */
#define PORT_INIT_SID (uint8)0x00

/* Service ID for Port Set Pin Direction */
#define PORT_SET_PIN_DIRECTION_SID (uint8)0x01

/* Service ID for Port Refresh Port Direction */
#define PORT_REFRESH_PORT_DIRECTION_SID (uint8)0x02

/* Service ID for Port Get Version Info */
#define PORT_GET_VERSION_INFO_SID (uint8)0x03

/* Service ID for Port Set Pin Mode */
#define PORT_SET_PIN_MODE_SID (uint8)0x04

/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
/* DET code to report Invalid Port Pin ID requested */
#define PORT_E_PARAM_PIN (uint8)0x0A

/* DET code to report Port Pin not configured as changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE (uint8)0x0B

/* DET code to report API Port_Init service called with wrong parameter */
#define PORT_E_INIT_FAILED (uint8)0x0C

/* DET code to report API Port_SetPinMode service called when mode passed is invalid */
#define PORT_E_PARAM_INVALID_MODE (uint8)0x0D

/* DET code to report API Port_SetPinMode service called when mode is unchangeable */
#define PORT_E_MODE_UNCHANGEABLE (uint8)0x0E

/* DET code to report API service called without module initialization */
#define PORT_E_UNINIT (uint8)0x0F

/* DET code to report APIs called with a Null Pointer */
#define PORT_E_PARAM_POINTER (uint8)0x10
/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

/* Type definition for Port_PinType used by the PORT APIs */
typedef uint8 Port_PinType;

/* Description: Enum to hold PIN direction */
typedef enum
{
  PORT_PIN_IN,
  PORT_PIN_OUT
} Port_PinDirectionType;

/* Description: Enum to hold PIN levels */
typedef enum
{
  PORT_PIN_LEVEL_LOW,
  PORT_PIN_LEVEL_HIGH
} Port_PinLevelValue;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
  OFF,
  PULL_UP,
  PULL_DOWN
} Port_InternalResistor;

/* Description: Enum to hold Digital Function (GPIOPCTL PMCx Bit Field Encoding) */
typedef enum
{
  /* GPIO Mode value = 0 */
  GPIO_MODE = 0,

  /* ADC (Analog) Mode value = 1 */
  ADC_MODE = 1U,

  /* Value = 1 */
  PA0_U0Rx = 1U,
  PA1_U0Tx = 1U,
  PB0_U1Rx = 1U,
  PB1_U1Tx = 1U,
  PC4_U4Rx = 1U,
  PC5_U4Tx = 1U,
  PC6_U3Rx = 1U,
  PC7_U4Tx = 1U,
  PD0_SSI3Clk = 1U,
  PD1_SSI3Fss = 1U,
  PD2_SSI3Rx = 1U,
  PD3_SSI3Tx = 1U,
  PD4_U6Rx = 1U,
  PD5_U6Tx = 1U,
  PD6_U2Rx = 1U,
  PD7_U2Tx = 1U,
  PE0_U7Rx = 1U,
  PE1_U7Tx = 1U,
  PE4_U5Rx = 1U,
  PE5_U5Tx = 1U,
  PF0_U1RTS = 1U,
  PF1_U1CTS = 1U,

  /* Value = 2 */
  PA2_SSI0Clk = 2U,
  PA3_SSI0Fss = 2U,
  PA4_SSI0Rx = 2U,
  PA5_SSI0Tx = 2U,
  PB4_SSI2Clk = 2U,
  PB5_SSI2Fss = 2U,
  PB6_SSI2Rx = 2U,
  PB7_SSI2Tx = 2U,
  PC4_U1Rx = 2U,
  PC5_U1Tx = 2U,
  PD0_SSI1Clk = 2U,
  PD1_SSI1Fss = 2U,
  PD2_SSI1Rx = 2U,
  PD3_SSI1Tx = 2U,
  PF0_SSI1Rx = 2U,
  PF1_SSI1Tx = 2U,
  PF2_SSI1Clk = 2U,
  PF3_SSI1Fss = 2U,

  /* Value = 3 */
  PA6_I2C1SCL = 3U,
  PA7_I2C1SDA = 3U,
  PB2_I2C0SCL = 3U,
  PB3_I2C0SDA = 3U,
  PD0_I2C3SCL = 3U,
  PD1_I2C3SDA = 3U,
  PE4_I2C2SCL = 3U,
  PE5_I2C2SDA = 3U,
  PF0_CAN0Rx = 3U,
  PF3_CAN0Tx = 3U,

  /* Value = 4 */
  PB4_M0PWM2 = 4U,
  PB5_M0PWM3 = 4U,
  PB6_M0PWM0 = 4U,
  PB7_M0PWM1 = 4U,
  PC4_M0PWM6 = 4U,
  PC5_M0PWM7 = 4U,
  PD0_M0PWM6 = 4U,
  PD1_M0PWM7 = 4U,
  PD2_M0FAULT0 = 4U,
  PD6_M0FAULT0 = 4U,
  PE4_M0PWM4 = 4U,
  PE5_M0PWM5 = 4U,
  PF2_M0FAULT0 = 4U,

  /* Value = 5 */
  PA6_M1PWM2 = 5U,
  PA7_M1PWM3 = 5U,
  PD0_M1PWM0 = 5U,
  PD1_M1PWM1 = 5U,
  PE4_M1PWM2 = 5U,
  PE5_M1PWM3 = 5U,
  PF0_M1PWM4 = 5U,
  PF1_M1PWM5 = 5U,
  PF2_M1PWM6 = 5U,
  PF3_M1PWM7 = 5U,
  PF4_M1FAULT0 = 5U,

  /* Value = 6 */
  PC4_IDX1 = 6U,
  PC5_PhA1 = 6U,
  PC6_PhB1 = 6U,
  PD3_IDX0 = 6U,
  PD6_PhA0 = 6U,
  PD7_PhB0 = 6U,
  PF0_PhA0 = 6U,
  PF1_PhB0 = 6U,
  PF4_IDX0 = 6U,

  /* Value = 7 */
  PB0_T2CCP0 = 7U,
  PB1_T2CCP1 = 7U,
  PB2_T3CCP0 = 7U,
  PB3_T3CCP1 = 7U,
  PB4_T1CCP0 = 7U,
  PB5_T1CCP1 = 7U,
  PB6_T0CCP0 = 7U,
  PB7_T0CCP1 = 7U,
  PC4_WT0CCP0 = 7U,
  PC5_WT0CCP1 = 7U,
  PC6_WT1CCP0 = 7U,
  PC7_WT1CCP1 = 7U,
  PD0_WT2CCP0 = 7U,
  PD1_WT2CCP1 = 7U,
  PD2_WT3CCP0 = 7U,
  PD3_WT3CCP1 = 7U,
  PD4_WT4CCP0 = 7U,
  PD5_WT4CCP1 = 7U,
  PD6_WT5CCP0 = 7U,
  PD7_WT5CCP1 = 7U,
  PF0_T0CCP0 = 7U,
  PF1_T0CCP1 = 7U,
  PF2_T1CCP0 = 7U,
  PF3_T1CCP1 = 7U,
  PF4_T2CCP0 = 7U,

  /* Value = 8 */
  PA0_CAN1Rx = 8U,
  PA1_CAN1Tx = 8U,
  PB4_CAN0Rx = 8U,
  PB5_CAN0Tx = 8U,
  PC4_U1RTS = 8U,
  PC5_U1CTS = 8U,
  PC6_USB0EPEN = 8U,
  PC7_USB0PFLT = 8U,
  PD2_USB0EPEN = 8U,
  PD3_USB0PFLT = 8U,
  PD7_NMI = 8U,
  PE4_CAN0Rx = 8U,
  PE5_CAN0Tx = 8U,
  PF0_NMI = 8U,
  PF4_USB0EPEN = 8U,

  /* Value = 9 */
  PF0_C0o = 9U,
  PF1_C1o = 9U,

  /* No modes with values 10-13 */

  /* Value = 14 */
  PF1_TRD1 = 14U,
  PF2_TRD0 = 14U,
  PF3_TRCLK = 14U
} Port_PinModeType;

/* Description: Type of the external data structure containing the initialization data for this module.
 *	1. The PORT Which the pin belongs to. eg. PortA, PortB etc..
 *	2. The number of the pin in the PORT. eg. Pin1, Pin2 etc..
 *  3. The direction of the pin. (Input or Output)
 *  4. The pin mode. eg. DIO, ADC, CAN, LIN etc..
 *  5. The internal resistor. Disabled, Pull up or Pull down
 *  6. The initial value of the pin. (High or Low)
 *  7. Pin direction changeable during runtime (STD_ON/STD_OFF)
 *  8. Pin mode changeable during runtime (STD_ON/STD_OFF)
 */
typedef struct
{
  uint8 portNumber;
  Port_PinType pinNumber;
  Port_PinDirectionType direction;
  Port_InternalResistor internalResistor;
  Port_PinLevelValue initialValue;
  Port_PinModeType mode;
  uint8 isDirectionChangeable;
  uint8 isModeChangeable;
} Port_ConfigSinglePinType;

typedef struct
{
  Port_ConfigSinglePinType pinConfig[PORT_NUMBER_OF_PINS];
} Port_ConfigType;

/* MCU Pin IDs */
#define PA0		(Port_PinType)0U
#define PA1		(Port_PinType)1U
#define PA2		(Port_PinType)2U
#define PA3		(Port_PinType)3U
#define PA4		(Port_PinType)4U
#define PA5		(Port_PinType)5U
#define PA6		(Port_PinType)6U
#define PA7		(Port_PinType)7U

#define PB0		(Port_PinType)8U
#define PB1		(Port_PinType)9U
#define PB2		(Port_PinType)10U
#define PB3		(Port_PinType)11U
#define PB4		(Port_PinType)12U
#define PB5		(Port_PinType)13U
#define PB6		(Port_PinType)14U
#define PB7		(Port_PinType)15U

#define PC0		(Port_PinType)16U
#define PC1		(Port_PinType)17U
#define PC2		(Port_PinType)18U
#define PC3		(Port_PinType)19U
#define PC4		(Port_PinType)20U
#define PC5		(Port_PinType)21U
#define PC6		(Port_PinType)22U
#define PC7		(Port_PinType)23U

#define PD0		(Port_PinType)24U
#define PD1		(Port_PinType)25U
#define PD2		(Port_PinType)26U
#define PD3		(Port_PinType)27U
#define PD4		(Port_PinType)28U
#define PD5		(Port_PinType)29U
#define PD6		(Port_PinType)30U
#define PD7		(Port_PinType)31U

#define PE0		(Port_PinType)32U
#define PE1		(Port_PinType)33U
#define PE2		(Port_PinType)34U
#define PE3		(Port_PinType)35U
#define PE4		(Port_PinType)36U
#define PE5		(Port_PinType)37U

#define PF0		(Port_PinType)38U
#define PF1		(Port_PinType)39U
#define PF2		(Port_PinType)40U
#define PF3		(Port_PinType)41U
#define PF4		(Port_PinType)42U
/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/
/* Initializes the Port Driver module */
void Port_Init(const Port_ConfigType *ConfigPtr);

/* Sets the port pin direction */
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(Port_PinType Pin, Port_PinDirectionType Direction);
#endif

/* Refreshes port direction */
void Port_RefreshPortDirection(void);

/* Returns the version information of this module */
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo);
#endif

/* Sets the port pin mode */
#if (PORT_SET_PIN_MODE == STD_ON)
void Port_SetPinMode(Port_PinType Pin, Port_PinModeType Mode);
#endif

/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

/* Extern PB structures to be used by Port and other modules */
extern const Port_ConfigType Port_PinConfigArray;

#endif /* PORT_H */
