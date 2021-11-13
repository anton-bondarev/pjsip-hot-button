
#include <stdint.h>

#include <drivers/audio/stm32h7_audio.h>


#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_i2s.h>

#include "audio_bsp.h"
#include "pdm2pcm_glo.h"


/* These PLL parameters are valid when the f(VCO clock) = 1Mhz */
const uint32_t I2SFreq[8] = {8000, 11025, 16000, 22050, 32000, 44100, 48000, 96000};
const uint32_t I2SPLLN[8] = {256, 429, 213, 429, 426, 271, 258, 344};
const uint32_t I2SPLLR[8] = {5, 4, 4, 4, 4, 6, 3, 1};

/*### RECORDER ###*/
I2S_HandleTypeDef                 hAudioOutI2s;

/*### RECORDER ###*/
I2S_HandleTypeDef                 hAudioInI2s;

/* PDM filters params */
PDM_Filter_Handler_t  PDM_FilterHandler[2];
PDM_Filter_Config_t   PDM_FilterConfig[2];

__IO uint16_t AudioInVolume = DEFAULT_AUDIO_IN_VOLUME;

static uint8_t I2S3_Init(uint32_t AudioFreq);
static uint8_t I2S2_Init(uint32_t AudioFreq);
static void PDMDecoder_Init(uint32_t AudioFreq, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut);




/**
 * @brief  Configures the audio peripherals.
 * @param  OutputDevice: OUTPUT_DEVICE_SPEAKER, OUTPUT_DEVICE_HEADPHONE,
 *                       OUTPUT_DEVICE_BOTH or OUTPUT_DEVICE_AUTO .
 * @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
 * @param  AudioFreq: Audio frequency used to play the audio stream.
 * @retval AUDIO_OK if correct communication, else wrong communication
 */
uint8_t BSP_AUDIO_OUT_Init(uint16_t OutputDevice, uint8_t Volume,
		uint32_t AudioFreq) {
	uint8_t ret = AUDIO_OK;

	/* PLL clock is set depending by the AudioFreq (44.1khz vs 48khz groups) */
	BSP_AUDIO_OUT_ClockConfig(&hAudioOutI2s, AudioFreq, NULL);

	/* I2S data transfer preparation:
	 Prepare the Media to be used for the audio transfer from memory to I2S peripheral */
	hAudioOutI2s.Instance = I2S3;
	if (HAL_I2S_GetState(&hAudioOutI2s) == HAL_I2S_STATE_RESET) {
		/* Init the I2S MSP: this __weak function can be redefined by the application*/
		BSP_AUDIO_OUT_MspInit(&hAudioOutI2s, NULL);
	}

	/* I2S data transfer preparation:
	 Prepare the Media to be used for the audio transfer from memory to I2S peripheral */
	/* Configure the I2S peripheral */
	if (I2S3_Init(AudioFreq) != AUDIO_OK) {
		ret = AUDIO_ERROR;
	}

	return ret;
}

/**
 * @brief  Starts playing audio stream from a data buffer for a determined size.
 * @param  pBuffer: Pointer to the buffer
 * @param  Size: Number of audio data BYTES.
 * @retval AUDIO_OK if correct communication, else wrong communication
 */
uint8_t BSP_AUDIO_OUT_Play(uint16_t *pBuffer, uint32_t Size) {

	/* Update the Media layer and enable it for play */
	HAL_I2S_Transmit_DMA(&hAudioOutI2s, pBuffer, DMA_MAX(Size / AUDIODATA_SIZE));

	/* Return AUDIO_OK when all operations are correctly done */
	return AUDIO_OK;
}

/**
 * @brief  Sends n-Bytes on the I2S interface.
 * @param  pData: Pointer to data address
 * @param  Size: Number of data to be written
 */
void BSP_AUDIO_OUT_ChangeBuffer(uint16_t *pData, uint16_t Size) {
	HAL_I2S_Transmit_DMA(&hAudioOutI2s, pData, Size);
}

/**
 * @brief   Pauses the audio file stream. In case of using DMA, the DMA Pause
 *          feature is used.
 * WARNING: When calling BSP_AUDIO_OUT_Pause() function for pause, only the
 *          BSP_AUDIO_OUT_Resume() function should be called for resume (use of BSP_AUDIO_OUT_Play()
 *          function for resume could lead to unexpected behavior).
 * @retval  AUDIO_OK if correct communication, else wrong communication
 */
uint8_t BSP_AUDIO_OUT_Pause(void) {
	/* Call the Media layer pause function */
	HAL_I2S_DMAPause(&hAudioOutI2s);

	/* Return AUDIO_OK when all operations are correctly done */
	return AUDIO_OK;
}

/**
 * @brief   Resumes the audio file streaming.
 * WARNING: When calling BSP_AUDIO_OUT_Pause() function for pause, only
 *          BSP_AUDIO_OUT_Resume() function should be called for resume (use of BSP_AUDIO_OUT_Play()
 *          function for resume could lead to unexpected behavior).
 * @retval  AUDIO_OK if correct communication, else wrong communication
 */
uint8_t BSP_AUDIO_OUT_Resume(void) {
		/* Call the Media layer resume function */
		HAL_I2S_DMAResume(&hAudioOutI2s);

		/* Return AUDIO_OK when all operations are correctly done */
		return AUDIO_OK;
}

/**
 * @brief  Stops audio playing and Power down the Audio Codec.
 * @param  Option: could be one of the following parameters
 *           - CODEC_PDWN_HW: completely shut down the codec (physically).
 *                            Then need to reconfigure the Codec after power on.
 * @retval AUDIO_OK if correct communication, else wrong communication
 */
uint8_t BSP_AUDIO_OUT_Stop(uint32_t Option) {
	/* Call DMA Stop to disable DMA stream before stopping codec */
	HAL_I2S_DMAStop(&hAudioOutI2s);
	return 0;
}

/**
 * @brief  Controls the current audio volume level.
 * @param  Volume: Volume level to be set in percentage from 0% to 100% (0 for
 *         Mute and 100 for Max volume level).
 * @retval AUDIO_OK if correct communication, else wrong communication
 */
uint8_t BSP_AUDIO_OUT_SetVolume(uint8_t Volume) {
	return 0;
}

/**
 * @brief  Enables or disables the MUTE mode by software
 * @param  Cmd: could be AUDIO_MUTE_ON to mute sound or AUDIO_MUTE_OFF to
 *         unmute the codec and restore previous volume level.
 * @retval AUDIO_OK if correct communication, else wrong communication
 */
uint8_t BSP_AUDIO_OUT_SetMute(uint32_t Cmd) {
	return 0;
}

/**
 * @brief  Update the audio frequency.
 * @param  AudioFreq: Audio frequency used to play the audio stream.
 * @note   This API should be called after the BSP_AUDIO_OUT_Init() to adjust the
 *         audio frequency.
 */
void BSP_AUDIO_OUT_SetFrequency(uint32_t AudioFreq) {
	/* PLL clock is set depending by the AudioFreq (44.1khz vs 48khz groups) */
	BSP_AUDIO_OUT_ClockConfig(&hAudioOutI2s, AudioFreq, NULL);

	/* Update the I2S audio frequency configuration */
	I2S3_Init(AudioFreq);
}

/**
 * @brief  Tx Transfer completed callbacks.
 * @param  hi2s: I2S handle
 */
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
	if (hi2s->Instance == I2S3) {
		/* Call the user function which will manage directly transfer complete */
		BSP_AUDIO_OUT_TransferComplete_CallBack();
	}
}

/**
 * @brief  Tx Half Transfer completed callbacks.
 * @param  hi2s: I2S handle
 */
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	if (hi2s->Instance == I2S3) {
		/* Manage the remaining file size and new address offset: This function should
		 be coded by user (its prototype is already declared in stm32f4_discovery_audio.h) */
		BSP_AUDIO_OUT_HalfTransfer_CallBack();
	}
}

/**
 * @brief  Clock Config.
 * @param  hi2s: might be required to set audio peripheral predivider if any.
 * @param  AudioFreq: Audio frequency used to play the audio stream.
 * @note   This API is called by BSP_AUDIO_OUT_Init() and BSP_AUDIO_OUT_SetFrequency()
 *         Being __weak it can be overwritten by the application
 * @param  Params : pointer on additional configuration parameters, can be NULL.
 */
__weak void BSP_AUDIO_OUT_ClockConfig(I2S_HandleTypeDef *hi2s,
		uint32_t AudioFreq, void *Params) {
	RCC_PeriphCLKInitTypeDef rccclkinit;
	uint8_t index = 0, freqindex = 0xFF;

	for (index = 0; index < 8; index++) {
		if (I2SFreq[index] == AudioFreq) {
			freqindex = index;
		}
	}
	/* Enable PLLI2S clock */
	HAL_RCCEx_GetPeriphCLKConfig(&rccclkinit);
	/* PLLI2S_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
	if ((freqindex & 0x7) == 0) {
		/* I2S clock config */
//		rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
//		rccclkinit.PLLI2S.PLLI2SN = I2SPLLN[freqindex];
//		rccclkinit.PLLI2S.PLLI2SR = I2SPLLR[freqindex];
		HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
	} else {
		/* I2S clock config */
//		rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
//		rccclkinit.PLLI2S.PLLI2SN = 258;
//		rccclkinit.PLLI2S.PLLI2SR = 3;
		HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
	}
}

/**
  * @brief  AUDIO OUT I2S MSP Init.
  * @param  hi2s: might be required to set audio peripheral predivider if any.
  * @param  Params : pointer on additional configuration parameters, can be NULL.
  */
__weak void BSP_AUDIO_OUT_MspInit(I2S_HandleTypeDef *hi2s, void *Params)
{
  static DMA_HandleTypeDef hdma_i2sTx;
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable I2S3 clock */
  I2S3_CLK_ENABLE();

  /*** Configure the GPIOs ***/
  /* Enable I2S GPIO clocks */
  I2S3_SCK_SD_CLK_ENABLE();
  I2S3_WS_CLK_ENABLE();

  /* I2S3 pins configuration: WS, SCK and SD pins ----------------------------*/
  GPIO_InitStruct.Pin         = I2S3_SCK_PIN | I2S3_SD_PIN;
  GPIO_InitStruct.Mode        = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull        = GPIO_NOPULL;
  GPIO_InitStruct.Speed       = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate   = I2S3_SCK_SD_WS_AF;
  HAL_GPIO_Init(I2S3_SCK_SD_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin         = I2S3_WS_PIN ;
  HAL_GPIO_Init(I2S3_WS_GPIO_PORT, &GPIO_InitStruct);

  /* I2S3 pins configuration: MCK pin */
  I2S3_MCK_CLK_ENABLE();
  GPIO_InitStruct.Pin         = I2S3_MCK_PIN;
  HAL_GPIO_Init(I2S3_MCK_GPIO_PORT, &GPIO_InitStruct);

  /* Enable the I2S DMA clock */
  I2S3_DMAx_CLK_ENABLE();

  if(hi2s->Instance == I2S3)
  {
    /* Configure the hdma_i2sTx handle parameters */
    //hdma_i2sTx.Init.Channel             = I2S3_DMAx_CHANNEL;
	hdma_i2sTx.Init.Request            = I2S3_DMAxREQUEST;
    hdma_i2sTx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_i2sTx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_i2sTx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_i2sTx.Init.PeriphDataAlignment = I2S3_DMAx_PERIPH_DATA_SIZE;
    hdma_i2sTx.Init.MemDataAlignment    = I2S3_DMAx_MEM_DATA_SIZE;
    hdma_i2sTx.Init.Mode                = DMA_NORMAL;
    hdma_i2sTx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_i2sTx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
    hdma_i2sTx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_i2sTx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_i2sTx.Init.PeriphBurst         = DMA_PBURST_SINGLE;

    hdma_i2sTx.Instance                 = I2S3_DMAx_STREAM;

    /* Associate the DMA handle */
    __HAL_LINKDMA(hi2s, hdmatx, hdma_i2sTx);

    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(&hdma_i2sTx);

    /* Configure the DMA Stream */
    HAL_DMA_Init(&hdma_i2sTx);
  }

  /* I2S DMA IRQ Channel configuration */
  HAL_NVIC_SetPriority(I2S3_DMAx_IRQ, AUDIO_OUT_IRQ_PREPRIO, 0);
  HAL_NVIC_EnableIRQ(I2S3_DMAx_IRQ);
}

/**
  * @brief  De-Initializes BSP_AUDIO_OUT MSP.
  * @param  hi2s: might be required to set audio peripheral predivider if any.
  * @param  Params : pointer on additional configuration parameters, can be NULL.
  */
__weak void BSP_AUDIO_OUT_MspDeInit(I2S_HandleTypeDef *hi2s, void *Params)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* I2S DMA IRQ Channel deactivation */
  HAL_NVIC_DisableIRQ(I2S3_DMAx_IRQ);

  if(hi2s->Instance == I2S3)
  {
    /* Deinitialize the Stream for new transfer */
    HAL_DMA_DeInit(hi2s->hdmatx);
  }

 /* Disable I2S block */
  __HAL_I2S_DISABLE(hi2s);

  /* CODEC_I2S pins configuration: SCK and SD pins */
  GPIO_InitStruct.Pin = I2S3_SCK_PIN | I2S3_SD_PIN;
  HAL_GPIO_DeInit(I2S3_SCK_SD_GPIO_PORT, GPIO_InitStruct.Pin);

  /* CODEC_I2S pins configuration: WS pin */
  GPIO_InitStruct.Pin = I2S3_WS_PIN;
  HAL_GPIO_DeInit(I2S3_WS_GPIO_PORT, GPIO_InitStruct.Pin);

  /* CODEC_I2S pins configuration: MCK pin */
  GPIO_InitStruct.Pin = I2S3_MCK_PIN;
  HAL_GPIO_DeInit(I2S3_MCK_GPIO_PORT, GPIO_InitStruct.Pin);

  /* Disable I2S clock */
  I2S3_CLK_DISABLE();

  /* GPIO pins clock and DMA clock can be shut down in the applic
     by surcgarging this __weak function */
}

/**
  * @brief  Manages the DMA full Transfer complete event.
  */
__weak void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
}

/**
  * @brief  Manages the DMA Half Transfer complete event.
  */
__weak void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{
}

/**
  * @brief  Manages the DMA FIFO error event.
  */
__weak void BSP_AUDIO_OUT_Error_CallBack(void)
{
}

/*******************************************************************************
                            Static Functions
*******************************************************************************/

/**
  * @brief  Initializes the Audio Codec audio interface (I2S).
  * @param  AudioFreq: Audio frequency to be configured for the I2S peripheral.
  */
static uint8_t I2S3_Init(uint32_t AudioFreq)
{
  /* Initialize the hAudioOutI2s Instance parameter */
  hAudioOutI2s.Instance         = I2S3;

 /* Disable I2S block */
  __HAL_I2S_DISABLE(&hAudioOutI2s);

  /* I2S3 peripheral configuration */
  hAudioOutI2s.Init.AudioFreq   = AudioFreq;
//  hAudioOutI2s.Init.ClockSource = I2S_CLOCK_PLL;
  hAudioOutI2s.Init.CPOL        = I2S_CPOL_LOW;
  hAudioOutI2s.Init.DataFormat  = I2S_DATAFORMAT_16B;
  hAudioOutI2s.Init.MCLKOutput  = I2S_MCLKOUTPUT_ENABLE;
  hAudioOutI2s.Init.Mode        = I2S_MODE_MASTER_TX;
//  hAudioOutI2s.Init.Standard    = I2S_STANDARD;
  /* Initialize the I2S peripheral with the structure above */
  if(HAL_I2S_Init(&hAudioOutI2s) != HAL_OK)
  {
    return AUDIO_ERROR;
  }
  else
  {
    return AUDIO_OK;
  }
}

/**************************************************
 * MICROPHONE
 **************************************************/














/**
  * @brief  Initializes wave recording.
  * @param  AudioFreq: Audio frequency to be configured for the I2S peripheral.
  * @param  BitRes: Audio frequency to be configured for the I2S peripheral.
  * @param  ChnlNbr: Audio frequency to be configured for the I2S peripheral.
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_IN_Init(uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr)
{

  /* Configure PLL clock */
  BSP_AUDIO_IN_ClockConfig(&hAudioInI2s, AudioFreq, NULL);

  /* Configure the PDM library */
  /* On STM32F4-Discovery a single microphone is mounted, samples are duplicated
     to make stereo audio streams */
  PDMDecoder_Init(AudioFreq, ChnlNbr, 2);

  /* Configure the I2S peripheral */
  hAudioInI2s.Instance = I2S2;
  if(HAL_I2S_GetState(&hAudioInI2s) == HAL_I2S_STATE_RESET)
  {
    /* Initialize the I2S Msp: this __weak function can be rewritten by the application */
    BSP_AUDIO_IN_MspInit(&hAudioInI2s, NULL);
  }

  /* Configure the I2S2 */
  I2S2_Init(AudioFreq);

  /* Return AUDIO_OK when all operations are correctly done */
  return AUDIO_OK;
}

/**
  * @brief  Starts audio recording.
  * @param  pbuf: Main buffer pointer for the recorded data storing
  * @param  size: Current size of the recorded buffer
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_IN_Record(uint16_t* pbuf, uint32_t size)
{
  uint32_t ret = AUDIO_ERROR;

  /* Start the process receive DMA */
  HAL_I2S_Receive_DMA(&hAudioInI2s, pbuf, size);

  /* Return AUDIO_OK when all operations are correctly done */
  ret = AUDIO_OK;

  return ret;
}

/**
  * @brief  Stops audio recording.
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_IN_Stop(void)
{
  uint32_t ret = AUDIO_ERROR;

  /* Call the Media layer pause function */
  HAL_I2S_DMAStop(&hAudioInI2s);

  /* Return AUDIO_OK when all operations are correctly done */
  ret = AUDIO_OK;

  return ret;
}

/**
  * @brief  Pauses the audio file stream.
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_IN_Pause(void)
{
  /* Call the Media layer pause function */
  HAL_I2S_DMAPause(&hAudioInI2s);

  /* Return AUDIO_OK when all operations are correctly done */
  return AUDIO_OK;
}

/**
  * @brief  Resumes the audio file stream.
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_IN_Resume(void)
{
  /* Call the Media layer pause/resume function */
  HAL_I2S_DMAResume(&hAudioInI2s);

  /* Return AUDIO_OK when all operations are correctly done */
  return AUDIO_OK;
}

/**
  * @brief  Controls the audio in volume level.
  * @param  Volume: Volume level to be set in percentage from 0% to 100% (0 for
  *         Mute and 100 for Max volume level).
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_IN_SetVolume(uint8_t Volume)
{
  /* Set the Global variable AudioInVolume */
  AudioInVolume = Volume;

  /* Return AUDIO_OK when all operations are correctly done */
  return AUDIO_OK;
}

/**
  * @brief  Converts audio format from PDM to PCM.
  * @param  PDMBuf: Pointer to data PDM buffer
  * @param  PCMBuf: Pointer to data PCM buffer
  * @retval AUDIO_OK if correct communication, else wrong communication
  */
uint8_t BSP_AUDIO_IN_PDMToPCM(uint16_t *PDMBuf, uint16_t *PCMBuf)
{
  uint16_t AppPDM[INTERNAL_BUFF_SIZE/2];
  uint32_t index = 0;

  /* PDM Demux */
  for(index = 0; index<INTERNAL_BUFF_SIZE/2; index++)
  {
    AppPDM[index] = HTONS(PDMBuf[index]);
  }

  for(index = 0; index < DEFAULT_AUDIO_IN_CHANNEL_NBR; index++)
  {
    /* PDM to PCM filter */
	PDM_Filter((uint8_t*)&AppPDM[index], (uint16_t*)&(PCMBuf[index]), &PDM_FilterHandler[index]);
  }
  /* Duplicate samples since a single microphone in mounted on STM32F4-Discovery */
  for(index = 0; index < PCM_OUT_SIZE; index++)
  {
    PCMBuf[(index<<1)+1] = PCMBuf[index<<1];
  }

  /* Return AUDIO_OK when all operations are correctly done */
  return AUDIO_OK;
}

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

/**
  * @brief  Audio In Clock Config.
  * @param  hi2s: I2S handle
  * @param  AudioFreq: Audio frequency used to record the audio stream.
  * @param  Params : pointer on additional configuration parameters, can be NULL.
  * @note   This API is called by BSP_AUDIO_IN_Init()
  *         Being __weak it can be overwritten by the application
  */
__weak void BSP_AUDIO_IN_ClockConfig(I2S_HandleTypeDef *hi2s,
		uint32_t AudioFreq, void *Params) {
	RCC_PeriphCLKInitTypeDef rccclkinit;

	/*Enable PLLI2S clock*/
	HAL_RCCEx_GetPeriphCLKConfig(&rccclkinit);
#if 0
  /* PLLI2S_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
  if ((AudioFreq & 0x7) == 0)
  {
    /* Audio frequency multiple of 8 (8/16/32/48/96/192)*/
    /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN = 192 Mhz */
    /* I2SCLK = PLLI2S_VCO Output/PLLI2SR = 192/6 = 32 Mhz */
//    rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
//    rccclkinit.PLLI2S.PLLI2SN = 192;
//    rccclkinit.PLLI2S.PLLI2SR = 6;
    HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
  }
  else
  {
    /* Other Frequency (11.025/22.500/44.100) */
    /* PLLI2S_VCO Output = PLLI2S_VCO Input * PLLI2SN = 290 Mhz */
    /* I2SCLK = PLLI2S_VCO Output/PLLI2SR = 290/2 = 145 Mhz */
//    rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_I2S;
//    rccclkinit.PLLI2S.PLLI2SN = 290;
//    rccclkinit.PLLI2S.PLLI2SR = 2;
    HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
  }
#endif
	rccclkinit.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
	rccclkinit.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
	HAL_RCCEx_PeriphCLKConfig(&rccclkinit);
}

/**
  * @brief  BSP AUDIO IN MSP Init.
  * @param  hi2s: I2S handle
  * @param  Params : pointer on additional configuration parameters, can be NULL.
  */
__weak void BSP_AUDIO_IN_MspInit(I2S_HandleTypeDef *hi2s, void *Params)
{
  static DMA_HandleTypeDef hdma_i2sRx;
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* Enable the I2S2 peripheral clock */
  I2S2_CLK_ENABLE();

  /* Enable I2S GPIO clocks */
  I2S2_SCK_GPIO_CLK_ENABLE();
  I2S2_MOSI_GPIO_CLK_ENABLE();
  I2S2_WS_GPIO_CLK_ENABLE();

  /* I2S2 pins configuration: SCK and MOSI pins ------------------------------*/
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;

  GPIO_InitStruct.Pin       = I2S2_SCK_PIN;
  GPIO_InitStruct.Alternate = I2S2_SCK_AF;
  HAL_GPIO_Init(I2S2_SCK_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin       = I2S2_MOSI_PIN ;
  GPIO_InitStruct.Alternate = I2S2_MOSI_AF;
  HAL_GPIO_Init(I2S2_MOSI_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin       = I2S2_WS_PIN ;
  GPIO_InitStruct.Alternate = I2S2_SCK_SD_WS_AF;
  HAL_GPIO_Init(I2S3_WS_GPIO_PORT, &GPIO_InitStruct);

  /* Enable the DMA clock */
  I2S2_DMAx_CLK_ENABLE();

  if(hi2s->Instance == I2S2)
  {
    /* Configure the hdma_i2sRx handle parameters */
    //hdma_i2sRx.Init.Channel             = I2S2_DMAx_CHANNEL;
	hdma_i2sRx.Init.Request            = I2S2_DMAxREQUEST;
    hdma_i2sRx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_i2sRx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_i2sRx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_i2sRx.Init.PeriphDataAlignment = I2S2_DMAx_PERIPH_DATA_SIZE;
    hdma_i2sRx.Init.MemDataAlignment    = I2S2_DMAx_MEM_DATA_SIZE;
    hdma_i2sRx.Init.Mode                = DMA_CIRCULAR;
    hdma_i2sRx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_i2sRx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_i2sRx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdma_i2sRx.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_i2sRx.Init.PeriphBurst         = DMA_MBURST_SINGLE;

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
  GPIO_InitTypeDef  gpio_init_structure;

  /* I2S DMA IRQ Channel deactivation */
  HAL_NVIC_DisableIRQ(I2S2_DMAx_IRQ);

  if(hi2s->Instance == I2S2)
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

/*******************************************************************************
                            Static Functions
*******************************************************************************/

/**
  * @brief  Initializes the PDM library.
  * @param  AudioFreq: Audio sampling frequency
  * @param  ChnlNbrIn: Number of input audio channels in the PDM buffer
  * @param  ChnlNbrOut: Number of desired output audio channels in the  resulting PCM buffer
  *         Number of audio channels (1: mono; 2: stereo)
  */
static void PDMDecoder_Init(uint32_t AudioFreq, uint32_t ChnlNbrIn, uint32_t ChnlNbrOut)
{
  uint32_t index = 0;

  /* Enable CRC peripheral to unlock the PDM library */
  __HAL_RCC_CRC_CLK_ENABLE();

  for(index = 0; index < ChnlNbrIn; index++)
  {
    /* Init PDM filters */
    PDM_FilterHandler[index].bit_order  = PDM_FILTER_BIT_ORDER_LSB;
    PDM_FilterHandler[index].endianness = PDM_FILTER_ENDIANNESS_LE;
    PDM_FilterHandler[index].high_pass_tap = 2122358088;
    PDM_FilterHandler[index].out_ptr_channels = ChnlNbrOut;
    PDM_FilterHandler[index].in_ptr_channels  = ChnlNbrIn;
    PDM_Filter_Init((PDM_Filter_Handler_t *)(&PDM_FilterHandler[index]));

    /* PDM lib config phase */
    PDM_FilterConfig[index].output_samples_number = AudioFreq/1000;
    PDM_FilterConfig[index].mic_gain = 24;
    PDM_FilterConfig[index].decimation_factor = PDM_FILTER_DEC_FACTOR_64;
    PDM_Filter_setConfig((PDM_Filter_Handler_t *)&PDM_FilterHandler[index], &PDM_FilterConfig[index]);
  }
}

/**
  * @brief  Initializes the Audio Codec audio interface (I2S)
  * @note   This function assumes that the I2S input clock (through PLL_R in
  *         Devices RevA/Z and through dedicated PLLI2S_R in Devices RevB/Y)
  *         is already configured and ready to be used.
  * @param  AudioFreq: Audio frequency to be configured for the I2S peripheral.
  */
static uint8_t I2S2_Init(uint32_t AudioFreq)
{
  /* Initialize the hAudioInI2s Instance parameter */
  hAudioInI2s.Instance          = I2S2;

  /* Disable I2S block */
  __HAL_I2S_DISABLE(&hAudioInI2s);

  /* I2S2 peripheral configuration */
  hAudioInI2s.Init.AudioFreq    = 2 * AudioFreq;
//  hAudioInI2s.Init.ClockSource  = I2S_CLOCK_PLL;
  hAudioInI2s.Init.CPOL         = I2S_CPOL_HIGH;
  hAudioInI2s.Init.DataFormat   = I2S_DATAFORMAT_16B;
  hAudioInI2s.Init.MCLKOutput   = I2S_MCLKOUTPUT_DISABLE;
  hAudioInI2s.Init.Mode         = I2S_MODE_MASTER_RX;
  hAudioInI2s.Init.Standard     = I2S_STANDARD_LSB;

  /* Initialize the I2S peripheral with the structure above */
  if(HAL_I2S_Init(&hAudioInI2s) != HAL_OK)
  {
    return AUDIO_ERROR;
  }
  else
  {
    return AUDIO_OK;
  }
}


/**
  * @brief  I2S error callbacks.
  * @param  hi2s: I2S handle
  */
void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s)
{
  /* Manage the error generated on DMA FIFO: This function
     should be coded by user (its prototype is already declared in stm32f4_discovery_audio.h) */
  if(hi2s->Instance == I2S3)
  {
    BSP_AUDIO_OUT_Error_CallBack();
  }
  if(hi2s->Instance == I2S2)
  {
    BSP_AUDIO_IN_Error_Callback();
  }
}
