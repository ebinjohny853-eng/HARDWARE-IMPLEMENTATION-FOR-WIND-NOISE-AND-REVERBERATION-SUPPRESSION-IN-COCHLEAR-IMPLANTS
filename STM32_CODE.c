/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Mic -> STFT -> Spectral Subtraction (with Bypass) -> IFFT -> DAC
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2S_BUFFER_SIZE 1024
#define SOUND_THRESHOLD 500000

/* --- OLA & FFT DEFINITIONS --- */
#define HOP_SIZE (I2S_BUFFER_SIZE / 4) // 256 samples per half-buffer
#define FFT_SIZE (HOP_SIZE * 2)        // 512 sample FFT for 50% overlap
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_tx;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Aligned Buffers for H7 Cache (Crucial for DMA) */
__attribute__((aligned(32))) int32_t rxBuffer[I2S_BUFFER_SIZE];
__attribute__((aligned(32))) int32_t txBuffer[I2S_BUFFER_SIZE];

volatile uint8_t processHalf = 0;
volatile uint8_t processFull = 0;

/* --- CMSIS DSP FFT VARIABLES --- */
arm_rfft_fast_instance_f32 fft_handler;

float32_t fft_input[FFT_SIZE];
float32_t fft_output[FFT_SIZE];
float32_t ifft_output[FFT_SIZE];

/* --- OVERLAP-ADD STATE ARRAYS --- */
float32_t hanning_window[FFT_SIZE];
float32_t audio_in_state[HOP_SIZE];   // Stores the previous 256 mic samples
float32_t audio_out_state[HOP_SIZE];  // Stores the overlapping 256 IFFT samples

/* --- SPECTRAL SUBTRACTION VARIABLES --- */
#define NOISE_ESTIMATION_FRAMES 50
#define SUBTRACTION_FACTOR 1.2f
#define SPECTRAL_FLOOR 0.02f

float32_t noise_spectrum[FFT_SIZE];
uint32_t noise_frame_count = 0;
uint8_t noise_estimation_done = 0;

/* --- BYPASS BUTTON VARIABLES --- */
volatile uint8_t noise_suppression_enabled = 1; // 1 = ON, 0 = OFF
uint32_t last_button_press = 0;                 // For debounce timer

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2S2_Init(void);
/* USER CODE BEGIN PFP */
void ProcessAudio(int32_t *srcBuffer, int32_t *dstBuffer);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* --- THE SOUND DETECTION & STFT/IFFT LOGIC --- */
void ProcessAudio(int32_t *srcBuffer, int32_t *dstBuffer)
{
  int32_t min_val = 2147483647;
  int32_t max_val = -2147483648;
  float32_t new_samples[HOP_SIZE];

  /* -------------------------------------------------------------------
   * STEP 1: EXTRACT NEW DATA & CONVERT TO FLOAT
   * ------------------------------------------------------------------- */
  uint16_t sample_idx = 0;
  for (int i = 0; i < (I2S_BUFFER_SIZE / 2); i += 2)
  {
    int32_t sample = srcBuffer[i];

    // Sign-Extend 24-bit audio to 32-bit
    if (sample & 0x00800000) {
        sample |= 0xFF000000;
    }

    // Track for LED logic
    if (sample > max_val) max_val = sample;
    if (sample < min_val) min_val = sample;

    // Convert to Float and scale
    new_samples[sample_idx++] = (float32_t)sample / 8388608.0f;
  }

  /* -------------------------------------------------------------------
   * STEP 2: BUILD 512-SAMPLE WINDOW (Old 256 + New 256)
   * ------------------------------------------------------------------- */
  for (int i = 0; i < HOP_SIZE; i++)
  {
    fft_input[i]            = audio_in_state[i];  // First half is old data
    fft_input[i + HOP_SIZE] = new_samples[i];     // Second half is new data

    // Save the new data into the state buffer for the NEXT interrupt
    audio_in_state[i] = new_samples[i];
  }

  /* -------------------------------------------------------------------
   * STEP 3: APPLY HANNING WINDOW
   * ------------------------------------------------------------------- */
  arm_mult_f32(fft_input, hanning_window, fft_input, FFT_SIZE);

  /* -------------------------------------------------------------------
   * STEP 4: FORWARD FFT -> FREQUENCY DOMAIN PROCESSING -> INVERSE FFT
   * ------------------------------------------------------------------- */
  arm_rfft_fast_f32(&fft_handler, fft_input, fft_output, 0);

  /* -------------------------------------------------------------------
   * SPECTRAL SUBTRACTION (WITH BYPASS LOGIC)
   * ------------------------------------------------------------------- */
  uint32_t num_bins = FFT_SIZE / 2;

  /* ---- Magnitude computation & noise estimation ---- */
  for (uint32_t k = 0; k < num_bins; k++)
  {
    float32_t real = fft_output[2*k];
    float32_t imag = fft_output[2*k + 1];
    float32_t mag  = sqrtf(real*real + imag*imag);

    if (!noise_estimation_done)
    {
      noise_spectrum[k] += mag;
    }
  }

  /* ---- Finalize noise profile ---- */
  if (!noise_estimation_done)
  {
    noise_frame_count++;

    if (noise_frame_count >= NOISE_ESTIMATION_FRAMES)
    {
      for (uint32_t k = 0; k < num_bins; k++)
      {
        noise_spectrum[k] /= (float32_t)NOISE_ESTIMATION_FRAMES;
      }
      noise_estimation_done = 1;
    }
  }

  /* ---- Apply spectral subtraction ONLY if enabled ---- */
  if (noise_estimation_done && noise_suppression_enabled)
  {
    for (uint32_t k = 0; k < num_bins; k++)
    {
      float32_t real = fft_output[2*k];
      float32_t imag = fft_output[2*k + 1];

      float32_t mag   = sqrtf(real*real + imag*imag);
      float32_t phase = atan2f(imag, real);

      float32_t clean_mag = mag - SUBTRACTION_FACTOR * noise_spectrum[k];

      if (clean_mag < SPECTRAL_FLOOR * noise_spectrum[k])
        clean_mag = SPECTRAL_FLOOR * noise_spectrum[k];

      fft_output[2*k]     = clean_mag * cosf(phase);
      fft_output[2*k + 1] = clean_mag * sinf(phase);
    }
  }

  arm_rfft_fast_f32(&fft_handler, fft_output, ifft_output, 1);

  /* -------------------------------------------------------------------
   * STEP 5: OVERLAP-ADD (OLA) RECONSTRUCTION TO DAC
   * ------------------------------------------------------------------- */
  sample_idx = 0;
  for (int i = 0; i < (I2S_BUFFER_SIZE / 2); i += 2)
  {
    // Add the first half of the new IFFT to the second half of the OLD IFFT
    float32_t out_f32 = ifft_output[sample_idx] + audio_out_state[sample_idx];

    // Save the second half of the new IFFT to be added NEXT time
    audio_out_state[sample_idx] = ifft_output[sample_idx + HOP_SIZE];

    // Convert Float back to 24-bit integer
    int32_t out_sample = (int32_t)(out_f32 * 8388608.0f);
    sample_idx++;

    // COPY TO DAC BUFFER (Play mono processed sound out of both L and R speakers)
    dstBuffer[i]   = out_sample;
    dstBuffer[i+1] = out_sample;
  }

  /* --- LED LOGIC --- */
  int32_t amplitude = max_val - min_val;
  if (amplitude > SOUND_THRESHOLD)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  }

  /* --- FLUSH CPU CACHE TO RAM SO DMA CAN TRANSMIT IT --- */
  SCB_CleanDCache_by_Addr((uint32_t *)dstBuffer, (I2S_BUFFER_SIZE / 2) * 4);
}

/* --- DMA INTERRUPTS --- */
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if (hi2s->Instance == SPI2) processHalf = 1;
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if (hi2s->Instance == SPI2) processFull = 1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S3_Init();
  MX_USART3_UART_Init();
  MX_I2S2_Init();
  /* USER CODE BEGIN 2 */

  /* --- INITIALIZE DSP AND STATE BUFFERS --- */
  arm_rfft_fast_init_f32(&fft_handler, FFT_SIZE);
  memset(audio_in_state, 0, sizeof(audio_in_state));
  memset(audio_out_state, 0, sizeof(audio_out_state));
  memset(noise_spectrum, 0, sizeof(noise_spectrum));

  /* Calculate the Hanning Window points dynamically */
  for(uint16_t i = 0; i < FFT_SIZE; i++)
  {
      // 0.5 * (1 - cos(2*PI*n / N))
      hanning_window[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * (float32_t)i / (float32_t)FFT_SIZE));
  }

  /* Clear Buffers */
  memset(rxBuffer, 0, sizeof(rxBuffer));
  memset(txBuffer, 0, sizeof(txBuffer));

  /* Flush the clean txBuffer to RAM before starting TX DMA */
  SCB_CleanDCache_by_Addr((uint32_t *)txBuffer, sizeof(txBuffer));

  /* Start DAC Playing (TX) */
  if (HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *)txBuffer, I2S_BUFFER_SIZE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start Mic Recording (RX) */
  if (HAL_I2S_Receive_DMA(&hi2s2, (uint16_t *)rxBuffer, I2S_BUFFER_SIZE) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* --- PROCESS FIRST HALF --- */
    if (processHalf)
    {
      processHalf = 0;
      SCB_InvalidateDCache_by_Addr((uint32_t *)&rxBuffer[0], (I2S_BUFFER_SIZE / 2) * 4);
      ProcessAudio(&rxBuffer[0], &txBuffer[0]);
    }

    /* --- PROCESS SECOND HALF --- */
    if (processFull)
    {
      processFull = 0;
      SCB_InvalidateDCache_by_Addr((uint32_t *)&rxBuffer[I2S_BUFFER_SIZE / 2], (I2S_BUFFER_SIZE / 2) * 4);
      ProcessAudio(&rxBuffer[I2S_BUFFER_SIZE / 2], &txBuffer[I2S_BUFFER_SIZE / 2]);
    }

    /* --- BUTTON DEBOUNCE CHECK (USER BUTTON ON PC13) --- */
    // If the Blue Button is pressed (Pin goes HIGH on Nucleo)
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET)
    {
      // If 300 milliseconds have passed since the last toggle (Debounce)
      if (HAL_GetTick() - last_button_press > 300)
      {
        // Toggle the state (0 becomes 1, 1 becomes 0)
        noise_suppression_enabled = !noise_suppression_enabled;

        // Record the time of this valid press
        last_button_press = HAL_GetTick();
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.FirstBit = I2S_FIRSTBIT_MSB;
  hi2s2.Init.WSInversion = I2S_WS_INVERSION_DISABLE;
  hi2s2.Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_RIGHT;
  hi2s2.Init.MasterKeepIOState = I2S_MASTER_KEEP_IO_STATE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.FirstBit = I2S_FIRSTBIT_MSB;
  hi2s3.Init.WSInversion = I2S_WS_INVERSION_DISABLE;
  hi2s3.Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_RIGHT;
  hi2s3.Init.MasterKeepIOState = I2S_MASTER_KEEP_IO_STATE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 (Nucleo Green LED) */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 (Nucleo Blue USER Button) */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
