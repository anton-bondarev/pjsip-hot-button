/*
 * audio_bsp.c
 *
 *  Created on: Aug 31, 2021
 *      Author: mvm
 */

#include <stdint.h>

#include <drivers/audio/stm32h7_audio.h>


#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_i2s.h>

#include "audio_bsp.h"
#include "pdm2pcm_glo.h"

/*### RECORDER ###*/
I2S_HandleTypeDef hi2s2;

#if 1
/* PDM filters params */
PDM_Filter_Handler_t PDM1_filter_handler;
PDM_Filter_Config_t PDM1_filter_config;
//static void PDMDecoder_Init(uint32_t AudioFreq, uint32_t ChnlNbrIn,
//		uint32_t ChnlNbrOut);
#endif

uint16_t AudioInVolume = DEFAULT_AUDIO_IN_VOLUME;

/**
 * @brief  Starts audio recording.
 * @param  pbuf: Main buffer pointer for the recorded data storing
 * @param  size: Current size of the recorded buffer
 * @retval AUDIO_OK if correct communication, else wrong communication
 */
uint8_t BSP_AUDIO_IN_RecordPDM(uint32_t Instance, uint8_t *pbuf, uint32_t size)
{
	uint32_t ret = AUDIO_ERROR;

	/* Start the process receive DMA */
	HAL_I2S_Receive_DMA(&hi2s2,(uint16_t *) pbuf, size);

	/* Return AUDIO_OK when all operations are correctly done */
	ret = AUDIO_OK;

	return ret;
}

/**
 * @brief  Stops audio recording.
 * @retval AUDIO_OK if correct communication, else wrong communication
 */
uint8_t BSP_AUDIO_IN_Stop(uint32_t Instance)
{
	uint32_t ret = AUDIO_ERROR;

	/* Call the Media layer pause function */
	HAL_I2S_DMAStop(&hi2s2);

	/* Return AUDIO_OK when all operations are correctly done */
	ret = AUDIO_OK;

	return ret;
}

/**
 * @brief  Pauses the audio file stream.
 * @retval AUDIO_OK if correct communication, else wrong communication
 */
uint8_t BSP_AUDIO_IN_Pause(uint32_t Instance)
{
	/* Call the Media layer pause function */
	HAL_I2S_DMAPause(&hi2s2);

	/* Return AUDIO_OK when all operations are correctly done */
	return AUDIO_OK;
}

/**
 * @brief  Resumes the audio file stream.
 * @retval AUDIO_OK if correct communication, else wrong communication
 */
uint8_t BSP_AUDIO_IN_Resume(uint32_t Instance)
{
	/* Call the Media layer pause/resume function */
	HAL_I2S_DMAResume(&hi2s2);

	/* Return AUDIO_OK when all operations are correctly done */
	return AUDIO_OK;
}

/**
 * @brief  Controls the audio in volume level.
 * @param  Volume: Volume level to be set in percentage from 0% to 100% (0 for
 *         Mute and 100 for Max volume level).
 * @retval AUDIO_OK if correct communication, else wrong communication
 */
#define MIC_MAX_VALUE (40)
#define MIC_MIN_VALUE (-12)
uint8_t BSP_AUDIO_IN_SetVolume(uint16_t Volume)
{
#if 0
	int16_t vol_db = MIC_MIN_VALUE
			+ (((float) MIC_MAX_VALUE - (float) MIC_MIN_VALUE) / 100) * Volume;
	if (vol_db < MIC_MIN_VALUE)
		vol_db = MIC_MIN_VALUE;
	else if (vol_db > MIC_MAX_VALUE)
		vol_db = MIC_MAX_VALUE;

	PDM1_filter_config.mic_gain = vol_db;
	PDM_Filter_setConfig((PDM_Filter_Handler_t*) &PDM1_filter_handler,
			&PDM1_filter_config);
	/* Return AUDIO_OK when all operations are correctly done */
#endif
	return AUDIO_OK;
}

#if 1
/**
 * @brief  Converts audio format from PDM to PCM.
 * @param  PDMBuf: Pointer to data PDM buffer
 * @param  PCMBuf: Pointer to data PCM buffer
 * @retval AUDIO_OK if correct communication, else wrong communication
 */
uint8_t BSP_AUDIO_IN_PDMToPCM(uint32_t Instance, uint16_t *PDMBuf, uint16_t *PCMBuf)
{

	PDM_Filter((uint8_t*) &PDMBuf[0], (uint16_t*) &PCMBuf[0],
			&PDM1_filter_handler);
	return AUDIO_OK;
}
#endif

#if 0
/**
 * @brief  Rx Transfer completed callbacks
 * @param  hi2s: I2S handle
 */
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	/* Call the record update function to get the next buffer to fill and its size (size is ignored) */
	BSP_AUDIO_IN_TransferComplete_CallBack();
}

/**
 * @brief  Rx Half Transfer completed callbacks.
 * @param  hi2s: I2S handle
 */
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	/* Manage the remaining file size and new address offset: This function
	 should be coded by user (its prototype is already declared in stm32f4_discovery_audio.h) */
	BSP_AUDIO_IN_HalfTransfer_CallBack();
}
#endif
/**
 * @brief  BSP AUDIO IN MSP Init.
 * @param  hi2s: I2S handle
 * @param  Params : pointer on additional configuration parameters, can be NULL.
 */
__weak void BSP_AUDIO_IN_MspInit(I2S_HandleTypeDef *hi2s, void *Params)
{
	static DMA_HandleTypeDef hdma_i2sRx;
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Enable the I2S2 peripheral clock */
	I2S2_CLK_ENABLE();

	/* Enable I2S GPIO clocks */
	I2S2_SCK_GPIO_CLK_ENABLE();
	I2S2_MOSI_GPIO_CLK_ENABLE();

	/* I2S2 pins configuration: SCK and MOSI pins ------------------------------*/
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

	GPIO_InitStruct.Pin = I2S2_SCK_PIN;
	GPIO_InitStruct.Alternate = I2S2_SCK_AF;
	HAL_GPIO_Init(I2S2_SCK_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = I2S2_MOSI_PIN;
	GPIO_InitStruct.Alternate = I2S2_MOSI_AF;
	HAL_GPIO_Init(I2S2_MOSI_GPIO_PORT, &GPIO_InitStruct);

	/* Enable the DMA clock */
	I2S2_DMAx_CLK_ENABLE();

	if (hi2s->Instance == I2S2)
	{
		/* Configure the hdma_i2sRx handle parameters */
		//hdma_i2sRx.Init.Channel = I2S2_DMAx_CHANNEL;
		hdma_i2sRx.Init.Request = I2S2_DMAxREQUEST;
		hdma_i2sRx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_i2sRx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_i2sRx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_i2sRx.Init.PeriphDataAlignment = I2S2_DMAx_PERIPH_DATA_SIZE;
		hdma_i2sRx.Init.MemDataAlignment = I2S2_DMAx_MEM_DATA_SIZE;
		hdma_i2sRx.Init.Mode = DMA_CIRCULAR;
		hdma_i2sRx.Init.Priority = DMA_PRIORITY_HIGH;
		hdma_i2sRx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		hdma_i2sRx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		hdma_i2sRx.Init.MemBurst = DMA_MBURST_SINGLE;
		hdma_i2sRx.Init.PeriphBurst = DMA_MBURST_SINGLE;

		hdma_i2sRx.Instance = I2S2_DMAx_STREAM;

		/* Associate the DMA handle */
		__HAL_LINKDMA(hi2s, hdmarx, hdma_i2sRx);

		/* Deinitialize the Stream for new transfer */
		HAL_DMA_DeInit(&hdma_i2sRx);

		/* Configure the DMA Stream */
		HAL_DMA_Init(&hdma_i2sRx);
	}

	/* I2S DMA IRQ Channel configuration */
	HAL_NVIC_SetPriority(I2S2_DMAx_IRQ, AUDIO_IN_IRQ_PREPRIO, 0);
	HAL_NVIC_EnableIRQ(I2S2_DMAx_IRQ);
}

/**
 * @brief  DeInitializes BSP_AUDIO_IN MSP.
 * @param  hi2s: I2S handle
 * @param  Params : pointer on additional configuration parameters, can be NULL.
 */
__weak void BSP_AUDIO_IN_MspDeInit(I2S_HandleTypeDef *hi2s, void *Params)
{
	GPIO_InitTypeDef gpio_init_structure;

	/* I2S DMA IRQ Channel deactivation */
	HAL_NVIC_DisableIRQ(I2S2_DMAx_IRQ);

	if (hi2s->Instance == I2S2)
	{
		/* Deinitialize the Stream for new transfer */
		HAL_DMA_DeInit(hi2s->hdmarx);
	}

	/* Disable I2S block */
	__HAL_I2S_DISABLE(hi2s);

	/* Disable pins: SCK and SD pins */
	gpio_init_structure.Pin = I2S2_SCK_PIN;
	HAL_GPIO_DeInit(I2S2_SCK_GPIO_PORT, gpio_init_structure.Pin);
	gpio_init_structure.Pin = I2S2_MOSI_PIN;
	HAL_GPIO_DeInit(I2S2_MOSI_GPIO_PORT, gpio_init_structure.Pin);

	/* Disable I2S clock */
	I2S2_CLK_DISABLE();

	/* GPIO pins clock and DMA clock can be shut down in the applic
	 by surcgarging this __weak function */
}

/**
 * @brief  User callback when record buffer is filled.
 */
__weak void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
	/* This function should be implemented by the user application.
	 It is called into this driver when the current buffer is filled
	 to prepare the next buffer pointer and its size. */
}
#if 0
/**
 * @brief  Manages the DMA Half Transfer complete event.
 */
__weak void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
	/* This function should be implemented by the user application.
	 It is called into this driver when the current buffer is filled
	 to prepare the next buffer pointer and its size. */
}

/**
 * @brief  Audio IN Error callback function.
 */
__weak void BSP_AUDIO_IN_Error_Callback(void)
{
	/* This function is called when an Interrupt due to transfer error on or peripheral
	 error occurs. */
}
#endif
void enable_amplifier()
{
	HAL_GPIO_WritePin(AMP_SD_GPIO_Port, AMP_SD_Pin, GPIO_PIN_SET);
}

void disable_amplifier()
{
	HAL_GPIO_WritePin(AMP_SD_GPIO_Port, AMP_SD_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  This function handles DMA Stream interrupt request.
 * @param  None
 * @retval None
 */
void I2S2_IRQHandler(void)
{
	HAL_DMA_IRQHandler(hi2s2.hdmarx);
}


int BSP_AUDIO_OUT_Play(uint32_t Instance, uint8_t *pData, uint32_t Size) {
	return 0;
}

int BSP_AUDIO_OUT_Pause(uint32_t Instance) {
	return 0;
}

int BSP_AUDIO_OUT_SetSampleRate(uint32_t Instance, uint32_t rate) {
	return 0;
}

int BSP_AUDIO_OUT_Resume(uint32_t Instance) {
	return 0;
}

int stm32h7_audio_init(void) {
	return 0;
}
