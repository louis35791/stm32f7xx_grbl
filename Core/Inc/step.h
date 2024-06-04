
#ifndef __STEP_H
#define __STEP_H

/* defines */
#define PULSE_WIDTH_US                      5UL // pulse width of HIGH state in microseconds                         
#define MINIMUN_LOW_PULSE_WIDTH_US          5UL // minimum pulse width of LOW state in microseconds
#define PULSE_WIDTH_TICKS                   (uint32_t)((PULSE_WIDTH_US * APB1_TIMER_CLOCK) / 1000000) // pulse width in ticks
#define MINIMUN_LOW_PULSE_WIDTH_TICKS       (uint32_t)((MINIMUN_LOW_PULSE_WIDTH_US * APB1_TIMER_CLOCK) / 1000000) // minimum pulse width in ticks
#define MINIMUM_PULSE_PERIOD_TICKS          (PULSE_WIDTH_TICKS + MINIMUN_LOW_PULSE_WIDTH_TICKS) // minimum pulse period in ticks

#define X_AXIS_TIM_BASE                     TIM2_BASE
#define Y_AXIS_TIM_BASE                     TIM2_BASE
#define Z_AXIS_TIM_BASE                     TIM5_BASE
#define X_AXIS_TIM                          ((TIM_TypeDef *)X_AXIS_TIM_BASE)
#define Y_AXIS_TIM                          ((TIM_TypeDef *)Y_AXIS_TIM_BASE)
#define Z_AXIS_TIM                          ((TIM_TypeDef *)Z_AXIS_TIM_BASE)
#define MASTER_TIM                          X_AXIS_TIM
#define X_AXIS_TIM_HANDLE                   &htim2
#define Y_AXIS_TIM_HANDLE                   &htim2
#define Z_AXIS_TIM_HANDLE                   &htim5
#define MASTER_TIM_HANDLE                   X_AXIS_TIM_HANDLE
#define X_AXIS_PULSE_TIM_ACTIVE_CHANNEL     HAL_TIM_ACTIVE_CHANNEL_3
#define Y_AXIS_PULSE_TIM_ACTIVE_CHANNEL     HAL_TIM_ACTIVE_CHANNEL_4
#define Z_AXIS_PULSE_TIM_ACTIVE_CHANNEL     HAL_TIM_ACTIVE_CHANNEL_1
#define X_AXIS_PULSE_TIM_DMA_ID             TIM_DMA_ID_CC3
#define Y_AXIS_PULSE_TIM_DMA_ID             TIM_DMA_ID_CC4
#define Z_AXIS_PULSE_TIM_DMA_ID             TIM_DMA_ID_CC1
#define X_AXIS_PULSE_TIM_CHANNEL            TIM_CHANNEL_3
#define Y_AXIS_PULSE_TIM_CHANNEL            TIM_CHANNEL_4
#define Z_AXIS_PULSE_TIM_CHANNEL            TIM_CHANNEL_1

#define GENERAL_NOTIFICATION_DATA_READY             0x01
#define GENERAL_NOTIFICATION_DATA_NOT_AVAILABLE     0x02

/* extern variables */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

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

// define ring buffer and associated variables
#define DEFINE_RING_BUFFER(__AXIS__, __BUFFER_SIZE__) \
    static doubleBufferArray_t __AXIS__##PulseBuffer[__BUFFER_SIZE__]; \
    // volatile uint16_t __AXIS__##RingBufferHead = 0; \
    // volatile uint16_t __AXIS__##RingBufferTail = 0; \

// get next index in ring buffer
#define RING_BUFFER_GET_NEXT(__AXIS__, __RING_BUFFER_SIZE__) \
    do  \
    {   \
        uint16_t nextIndex = (__AXIS__##RingBufferHead + 1) % (__RING_BUFFER_SIZE__); \
        if (nextIndex != __AXIS__##RingBufferTail) \
        { \
            return nextIndex; \
        } \
        else return UINT16_MAX; \
    } while (0); \

// increment ring buffer head
#define RING_BUFFER_INCREMENT_HEAD(__AXIS__, __RING_BUFFER_SIZE__) \
    do  \
    {   \
        uint16_t nextIndex = (__AXIS__##RingBufferHead + 1) % (__RING_BUFFER_SIZE__); \
        if (nextIndex != __AXIS__##RingBufferTail) \
        { \
            __AXIS__##RingBufferHead = nextIndex; \
        } \
    } while (0); \

// get ring buffer tail
#define RING_BUFFER_GET_TAIL(__AXIS__) \
    do  \
    {   \
        if (__AXIS__##RingBufferTail != __AXIS__##RingBufferHead) \
        { \
            return __AXIS__##RingBufferTail; \
        } \
        else return UINT16_MAX; \
    } while (0); \
    
// increment ring buffer tail
#define RING_BUFFER_INCREMENT_TAIL(__AXIS__, __RING_BUFFER_SIZE__) \
    do  \
    {   \
        if (__AXIS__##RingBufferTail != __AXIS__##RingBufferHead) \
        { \
            __AXIS__##RingBufferTail = (__AXIS__##RingBufferTail + 1) % (__RING_BUFFER_SIZE__); \
        } \
    } while (0); \

// get which axis the time handle belongs to
#define GET_AXIS_FROM_TIM_HANDLE(__TIM_HANDLE__) \
    (((__TIM_HANDLE__ == X_AXIS_TIM_HANDLE) && (__TIM_HANDLE__->Channel == X_AXIS_PULSE_TIM_ACTIVE_CHANNEL)) ? X_AXIS : \
     ((__TIM_HANDLE__ == Y_AXIS_TIM_HANDLE) && (__TIM_HANDLE__->Channel == Y_AXIS_PULSE_TIM_ACTIVE_CHANNEL)) ? Y_AXIS : \
     ((__TIM_HANDLE__ == Z_AXIS_TIM_HANDLE) && (__TIM_HANDLE__->Channel == Z_AXIS_PULSE_TIM_ACTIVE_CHANNEL)) ? Z_AXIS : \
    NUM_DIMENSIONS) \

// get the index of specified axis in DMA handlers array
#define GET_TIM_DMA_ID_FROM_AXIS(__AXIS__) \
    ((__AXIS__ == X_AXIS) ? X_AXIS_PULSE_TIM_DMA_ID : \
     (__AXIS__ == Y_AXIS) ? Y_AXIS_PULSE_TIM_DMA_ID : \
     (__AXIS__ == Z_AXIS) ? Z_AXIS_PULSE_TIM_DMA_ID : \
     UINT8_MAX) \

// check if ring buffer is exhausted
#define IS_RING_BUFFER_EXHAUSTED(__AXIS__) (__AXIS__##RingBufferHead == __AXIS__##RingBufferTail)


/* exported functions */
void stepTask(void *pvParameters);

#endif // __STEP_H