/*
 * audio_bsp.h
 *
 *  Created on: Aug 31, 2021
 *      Author: mvm
 */

#ifndef SRC_AUDIO_AUDIO_BSP_H_
#define SRC_AUDIO_AUDIO_BSP_H_


#define AMP_SD_Pin GPIO_PIN_15
#define AMP_SD_GPIO_Port GPIOB

#include <stdint.h>

#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_i2s.h>

//#include "main.h"
/*------------------------------------------------------------------------------
 AUDIO IN CONFIGURATION
 ------------------------------------------------------------------------------*/
/* SPI Configuration defines */
#define I2S2                            SPI2
#define I2S2_CLK_ENABLE()               __HAL_RCC_SPI2_CLK_ENABLE()
#define I2S2_CLK_DISABLE()              __HAL_RCC_SPI2_CLK_DISABLE()
#define I2S2_SCK_PIN                    GPIO_PIN_10
#define I2S2_SCK_GPIO_PORT              GPIOB
#define I2S2_SCK_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2S2_SCK_AF                     GPIO_AF5_SPI2

#define I2S2_MOSI_PIN                   GPIO_PIN_3
#define I2S2_MOSI_GPIO_PORT             GPIOC
#define I2S2_MOSI_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()
#define I2S2_MOSI_AF                    GPIO_AF5_SPI2

/* I2S DMA Stream Rx definitions */
#define I2S2_DMAx_CLK_ENABLE()          __HAL_RCC_DMA1_CLK_ENABLE()
#define I2S2_DMAx_CLK_DISABLE()         __HAL_RCC_DMA1_CLK_DISABLE()
#define I2S2_DMAx_STREAM                DMA1_Stream3
//#define I2S2_DMAx_CHANNEL               DMA_CHANNEL_0
#define I2S2_DMAxREQUEST                DMA_REQUEST_SPI2_RX
#define I2S2_DMAx_IRQ                   DMA1_Stream3_IRQn
#define I2S2_DMAx_PERIPH_DATA_SIZE      DMA_PDATAALIGN_HALFWORD
#define I2S2_DMAx_MEM_DATA_SIZE         DMA_MDATAALIGN_HALFWORD

#define I2S2_IRQHandler                 DMA1_Stream0_IRQHandler

/* Select the interrupt preemption priority and subpriority for the IT/DMA interrupt */
#define AUDIO_IN_IRQ_PREPRIO            0x0F   /* Select the preemption priority level(0 is the highest) */

/** @defgroup STM32F4_DISCOVERY_AUDIO_IN_Exported_Functions STM32F4 DISCOVERY AUDIO IN Exported Functions
 * @{
 */
uint8_t BSP_AUDIO_IN_Init(uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr);
uint8_t BSP_AUDIO_IN_RecordPDM(uint32_t Instance, uint8_t *pData, uint32_t Size);
uint8_t BSP_AUDIO_IN_Stop(uint32_t Instance);
uint8_t BSP_AUDIO_IN_Pause(uint32_t Instance);
uint8_t BSP_AUDIO_IN_Resume(uint32_t Instance);
uint8_t BSP_AUDIO_IN_SetVolume(uint16_t Volume);
uint8_t BSP_AUDIO_IN_PDMToPCM(uint32_t Instance, uint16_t *PDMBuf, uint16_t *PCMBuf);
/* User Callbacks: user has to implement these functions in his code if they are needed. */
/* This function should be implemented by the user application.
 It is called into this driver when the current buffer is filled to prepare the next
 buffer pointer and its size. */
//void BSP_AUDIO_IN_TransferComplete_CallBack(void);
//void BSP_AUDIO_IN_HalfTransfer_CallBack(void);

/* This function is called when an Interrupt due to transfer error on or peripheral
 error occurs. */
void BSP_AUDIO_IN_Error_Callback(void);

/* These function can be modified in case the current settings (e.g. DMA stream)
 need to be changed for specific application needs */
void BSP_AUDIO_IN_ClockConfig(I2S_HandleTypeDef *hi2s, uint32_t AudioFreq,
		void *Params);
void BSP_AUDIO_IN_MspInit(I2S_HandleTypeDef *hi2s, void *Params);
void BSP_AUDIO_IN_MspDeInit(I2S_HandleTypeDef *hi2s, void *Params);

/* Audio status definition */
#define AUDIO_OK                        0
#define AUDIO_ERROR                     1
#define AUDIO_TIMEOUT                   2

/* AudioFreq * DataSize (2 bytes) * NumChannels (Stereo: 2) */
#define DEFAULT_AUDIO_IN_FREQ                 I2S_AUDIOFREQ_16K
#define DEFAULT_AUDIO_IN_BIT_RESOLUTION       16
#define DEFAULT_AUDIO_IN_CHANNEL_NBR          1 /* Mono = 1, Stereo = 2 */
#define DEFAULT_AUDIO_IN_VOLUME               64

/* PDM buffer input size */
#define INTERNAL_BUFF_SIZE                    128*DEFAULT_AUDIO_IN_FREQ/16000*DEFAULT_AUDIO_IN_CHANNEL_NBR
/* PCM buffer output size */
#define PCM_OUT_SIZE                          DEFAULT_AUDIO_IN_FREQ/1000
#define CHANNEL_DEMUX_MASK                    0x55

#define DMA_MAX(_X_)                (((_X_) <= DMA_MAX_SZE)? (_X_):DMA_MAX_SZE)
#define HTONS(A)  ((((uint16_t)(A) & 0xff00) >> 8) | (((uint16_t)(A) & 0x00ff) << 8))



extern int BSP_AUDIO_OUT_Play(uint32_t Instance, uint8_t *pData, uint32_t Size);
extern int BSP_AUDIO_OUT_Pause(uint32_t Instance);
extern int BSP_AUDIO_OUT_SetSampleRate(uint32_t Instance, uint32_t rate);
extern int BSP_AUDIO_OUT_Resume(uint32_t Instance);


extern int stm32h7_audio_init(void);

void enable_amplifier();
void disable_amplifier();

#endif /* SRC_AUDIO_AUDIO_BSP_H_ */
