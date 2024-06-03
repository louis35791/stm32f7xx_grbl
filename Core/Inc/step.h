
#ifndef __STEP_H
#define __STEP_H

/* defines */
#define X_AXIS_TIM_BASE                     TIM2_BASE
#define Y_AXIS_TIM_BASE                     TIM3_BASE
#define Z_AXIS_TIM_BASE                     TIM4_BASE
#define X_AXIS_TIM                          ((TIM_TypeDef *)X_AXIS_TIM_BASE)
#define Y_AXIS_TIM                          ((TIM_TypeDef *)Y_AXIS_TIM_BASE)
#define Z_AXIS_TIM                          ((TIM_TypeDef *)Z_AXIS_TIM_BASE)
#define X_AXIS_TIM_HANDLE                   &htim2
#define Y_AXIS_TIM_HANDLE                   &htim3
#define Z_AXIS_TIM_HANDLE                   &htim4
#define X_AXIS_PULSE_TIM_ACTIVE_CHANNEL     HAL_TIM_ACTIVE_CHANNEL_3
#define X_AXIS_DIR_TIM_ACTIVE_CHANNEL       HAL_TIM_ACTIVE_CHANNEL_4
#define Y_AXIS_PULSE_TIM_ACTIVE_CHANNEL     HAL_TIM_ACTIVE_CHANNEL_1
#define Y_AXIS_DIR_TIM_ACTIVE_CHANNEL       HAL_TIM_ACTIVE_CHANNEL_2
#define Z_AXIS_PULSE_TIM_ACTIVE_CHANNEL     HAL_TIM_ACTIVE_CHANNEL_1
#define Z_AXIS_DIR_TIM_ACTIVE_CHANNEL       HAL_TIM_ACTIVE_CHANNEL_2
#define X_AXIS_PULSE_TIM_DMA_ID             TIM_DMA_ID_CC3
#define X_AXIS_DIR_TIM_DMA_ID               TIM_DMA_ID_CC4
#define Y_AXIS_PULSE_TIM_DMA_ID             TIM_DMA_ID_CC1
#define Y_AXIS_DIR_TIM_DMA_ID               TIM_DMA_ID_CC2
#define Z_AXIS_PULSE_TIM_DMA_ID             TIM_DMA_ID_CC1
#define Z_AXIS_DIR_TIM_DMA_ID               TIM_DMA_ID_CC2
#define X_AXIS_PULSE_TIM_CHANNEL            TIM_CHANNEL_3
#define X_AXIS_DIR_TIM_CHANNEL              TIM_CHANNEL_4
#define Y_AXIS_PULSE_TIM_CHANNEL            TIM_CHANNEL_1
#define Y_AXIS_DIR_TIM_CHANNEL              TIM_CHANNEL_2
#define Z_AXIS_PULSE_TIM_CHANNEL            TIM_CHANNEL_1
#define Z_AXIS_DIR_TIM_CHANNEL              TIM_CHANNEL_2

#define GENERAL_NOTIFICATION_DATA_READY             0x01
#define GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE     0x02

/* extern variables */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* type define */
typedef enum AxisEnum
{
    X_AXIS = 0,
    Y_AXIS,
    Z_AXIS,
    NUM_DIMENSIONS
} axis_t;

/**
 * NOTE: Here we use uint32_t instead of uint16_t as the format of the buffer
 *       in case the 32-bit timer does not support 16-bit DMA.
 *       There is an unexpected result occurred as uint16_t is used,
 *       which ends up with two replicates of 16-bit data concatenated
 *       into a single 32-bit register.
 */
typedef uint32_t doubleBufferArray_t[DOUBLE_BUFFER_SIZE];

typedef struct
{
    __IO uint32_t ISR; /*!< DMA interrupt status register */
    __IO uint32_t Reserved0;
    __IO uint32_t IFCR; /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;

/* Macros */
#define CONCATENATE(A, B) A##B

#define COMPUTE_INCREMENT_FOR_PULSE(pulse_freq, clock_freq) ((uint16_t)((uint32_t)clock_freq / ((uint32_t)pulse_freq * 2)))

// get TIM capture compare register value on runtime
#define TIM_GET_COMPARE(__HANDLE__, __ACTIVE_CHANNEL__) \
  (((__ACTIVE_CHANNEL__) == HAL_TIM_ACTIVE_CHANNEL_1) ? ((__HANDLE__)->Instance->CCR1) :\
   ((__ACTIVE_CHANNEL__) == HAL_TIM_ACTIVE_CHANNEL_2) ? ((__HANDLE__)->Instance->CCR2) :\
   ((__ACTIVE_CHANNEL__) == HAL_TIM_ACTIVE_CHANNEL_3) ? ((__HANDLE__)->Instance->CCR3) :\
   ((__ACTIVE_CHANNEL__) == HAL_TIM_ACTIVE_CHANNEL_4) ? ((__HANDLE__)->Instance->CCR4) :\
   ((__ACTIVE_CHANNEL__) == HAL_TIM_ACTIVE_CHANNEL_5) ? ((__HANDLE__)->Instance->CCR5) :\
   ((__HANDLE__)->Instance->CCR6))

// suspend DMA stream
#define SUSPEND_DMA_STREAM(__DMA_HANDLE__) __HAL_DMA_DISABLE((__DMA_HANDLE__))

// clear all DMA interrupt flags
// __DMA_HANDLE__ is the DMA_HandleTypeDef structure
#define CLEAR_DMA_IT(__DMA_HANDLE__) \
    do                               \
    {                                \
        DMA_Base_Registers *regs = (DMA_Base_Registers *)(__DMA_HANDLE__)->StreamBaseAddress; \
        regs->IFCR = 0x3FU << (__DMA_HANDLE__)->StreamIndex; \
    } while (0);

// Resume DMA stream with transfer complete interrupt being set in CR
#define RESUME_DMA_STREAM_WITH_TC(__DMA_HANDLE__, __BUFFER__, __LENGTH__) \
    do                                                                    \
    {                                                                     \
        CLEAR_DMA_IT(__DMA_HANDLE__);                                     \
        (__DMA_HANDLE__)->Instance->M0AR = (uint32_t)(__BUFFER__);        \
        (__DMA_HANDLE__)->Instance->NDTR = (uint32_t)(__LENGTH__);        \
        (__DMA_HANDLE__)->Instance->CR |= DMA_IT_TC;                      \
        __HAL_DMA_ENABLE((__DMA_HANDLE__));                               \
    } while (0);

// Timer start counter
#define TIM_START_COUNTER(__HANDLE__) ((__HANDLE__)->CR1 |= TIM_CR1_CEN)

// Timer stop counter
#define TIM_STOP_COUNTER(__HANDLE__) ((__HANDLE__)->CR1 &= ~TIM_CR1_CEN)

// swap buffer
#define SWAP_BUFFER_UINT32(__BUFFER1__, __BUFFER2__) \
    do                                               \
    {                                                \
        doubleBufferArray_t *temp = __BUFFER1__;                \
        __BUFFER1__ = __BUFFER2__;                   \
        __BUFFER2__ = temp;                          \
    } while (0);

// initialize TIM_DMA_Parameters_t
#define INIT_TIM_DMA_PARAMETERS(__TARGET_ARR__, __AXIS__)   \
    do                                      \
    {                                       \
        (__TARGET_ARR__)[__AXIS__].pulse.htim = CONCATENATE(__AXIS__, _TIM_HANDLE); \
        (__TARGET_ARR__)[__AXIS__].pulse.TIM_DMA_ID = CONCATENATE(__AXIS__, _PULSE_TIM_DMA_ID); \
        (__TARGET_ARR__)[__AXIS__].pulse.TIM_DMA_ID_CC= CONCATENATE(__AXIS__, _PULSE_TIM_ACTIVE_CHANNEL); \
        (__TARGET_ARR__)[__AXIS__].pulse.TIM_CHANNEL= CONCATENATE(__AXIS__, _PULSE_TIM_CHANNEL); \
        (__TARGET_ARR__)[__AXIS__].pulse.lengthAvailableData= 0; \
        (__TARGET_ARR__)[__AXIS__].dir.htim = CONCATENATE(__AXIS__, _TIM_HANDLE); \
        (__TARGET_ARR__)[__AXIS__].dir.TIM_DMA_ID = CONCATENATE(__AXIS__, _DIR_TIM_DMA_ID); \
        (__TARGET_ARR__)[__AXIS__].dir.TIM_DMA_ID_CC= CONCATENATE(__AXIS__, _DIR_TIM_ACTIVE_CHANNEL); \
        (__TARGET_ARR__)[__AXIS__].dir.TIM_CHANNEL= CONCATENATE(__AXIS__, _DIR_TIM_CHANNEL); \
        (__TARGET_ARR__)[__AXIS__].dir.lengthAvailableData= 0; \
    } while (0);

/* exported functions */
void vStepTask(void *pvParameters);
HAL_StatusTypeDef TIM_OC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, const uint32_t *pData, uint16_t Length);

#endif // __STEP_H