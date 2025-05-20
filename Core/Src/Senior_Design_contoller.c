/*******************************************************************************
  * This file demonstrates how debouncing can be implemented on a button
  * connected to PC6 and GND, using Timer7 for debouncing.
  * While inputs are being ignored (debouncing), LED LD6 will be turned on.
  * Each time the PC6 button is pressed, LED LD3 toggles.
  * The User Button on PA0 is configured to trigger an interrupt, but currently
  * does not perform any action when pressed.
  * Timer6 is also configured but no functionality is associated with its
  * expiration in this example.
  *
  * In this version, interrupt handlers are placed in this main.c file instead
  * of stm32f3xx_it.c (where they are usually located).
  * The system clock is configured to 64 MHz, which drives the timers.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <main_basic_timer_example_debouncing.h>
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
// TODO: Variables related to LED blinking for Lab 2:
#define ONE_SEC_HALF_PERIOD 1000 // Adjust this for the desired period.
#define TWO_SEC_HALF_PERIOD 2000 // Adjust this for the desired period.
#define THREE_SEC_HALF_PERIOD 3000 // Adjust this for the desired period.

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
// Handles for Timer6 and Timer7:
TIM_HandleTypeDef my_tim6_handle;
TIM_HandleTypeDef my_tim7_handle;

// State variable to track if inputs are being ignored for debouncing:
static volatile bool ignoringInputs = false;

static volatile uint32_t buttonpresscount = 0;

static volatile bool is_flashing = false;

typedef enum {fast, medium, slow} machine_state_t;
static volatile machine_state_t blinking_speed;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void Configure_Timer6(void);
static void Configure_Timer7(void);

/* Private user code ---------------------------------------------------------*/

// Note: IRQHandler functions are usually located in Core > Src > stm32f3xx_it.c,
// but they have been moved here for ease of reference in this example.
/**
  * @brief This function handles EXTI line0 interrupt.
  */
// Called when an interrupt occurs on one of PA0, PB0, etc.
void EXTI0_IRQHandler(void) {
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
// A single interrupt handler function for Pins 5-9.
// The pin number that triggered the interrupt is passed to the HAL function.
// Note: the Callback function also receives the Pin number as an argument.
void EXTI9_5_IRQHandler(void) {
	// Handle the interrupt triggered by PC6, which is connected to Pin6.
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
}

// Callback function triggered by the interrupt.
// It verifies which pin triggered the interrupt.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// For Pin0 (PA0), no action is taken (this can be changed in Lab 2).
	if (GPIO_Pin == GPIO_PIN_0) {

		switch(blinking_speed) {
						case fast:
							// Turn on LED LD8 and set Timer6 for a 1-second period.
							HAL_GPIO_WritePin(LD8_GPIO_Port, LD8_Pin, GPIO_PIN_SET);
							HAL_GPIO_WritePin(LD9_GPIO_Port, LD9_Pin, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(LD10_GPIO_Port, LD10_Pin, GPIO_PIN_RESET);


							// Transition to the medium speed state.
							blinking_speed = slow;
							break;

						case slow:
							// Change the LEDs and set Timer6 for a 3-second period.
							HAL_GPIO_WritePin(LD8_GPIO_Port, LD8_Pin, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(LD9_GPIO_Port, LD9_Pin, GPIO_PIN_SET);
							HAL_GPIO_WritePin(LD10_GPIO_Port, LD10_Pin, GPIO_PIN_RESET);


							// Transition back to the fast speed state.
							blinking_speed = fast;
							break;

						default:
							// Handle error: perhaps by turning on an error LED.
					}

	}
	else if (GPIO_Pin == GPIO_PIN_6) {

		// If inputs are being ignored (debouncing), do nothing.
		if (ignoringInputs) {
			return;
		} else {
			// Handle the first rising edge detected on the PC6 button.
			// Currently, it toggles LD3, but more functionality can be added for Lab 2.


			// Turn on the Timer7 indicator LED (LD6) to show that inputs are being ignored.

			// Start Timer7 to ignore inputs for a while (debouncing).
			HAL_TIM_Base_Start_IT(&my_tim7_handle);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

			// Transition to ignoring inputs.
			ignoringInputs = true;
		}
	}
}

// Timer6 interrupt handler.
void TIM6_DAC_IRQHandler(void) {
  HAL_TIM_IRQHandler(&my_tim6_handle);
}

// Timer7 interrupt handler.
void TIM7_IRQHandler(void) {
  HAL_TIM_IRQHandler(&my_tim7_handle);
}

// Period elapsed callback for timers.
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
	  // Toggle LED LD3 if flashing is enabled.

  }
  else if (htim->Instance == TIM7) {
	  // Stop Timer7 and turn off LD6 when Timer7 expires (end of debouncing).
	  HAL_TIM_Base_Stop_IT(&my_tim7_handle);
	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);

	  // Transition out of the ignoring inputs state (debouncing ends).
	  ignoringInputs = false;
  }
}
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{


  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  // Also Turn of LD6 to begin with because Timer7 isn't running:
  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);


  // Configure Timer 6:
  Configure_Timer6();
  // Start Timer6 in "Interrupt" mode so it generates an interrupt
  // when it goes off:
  //HAL_TIM_Base_Start_IT(&my_tim6_handle);
  // In this example, we don't do anything with Timer6.  This line of
  // code is how you start it.

  // Configure Timer 7 to generate a single pulse:
  Configure_Timer7();
  // We will only start this when you push the PC6 Button and we need
  // to debounce it.


	__HAL_TIM_SET_AUTORELOAD(&my_tim6_handle, ONE_SEC_HALF_PERIOD);

  /* Infinite loop */
  while (1)
  {
	  // Note we do not do ANYTHING AT ALL inside the while(1) loop!
	  // We can just make the processor go to sleep to save power if we want.

  }

}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  // The PLL is 8 MHz, and it is multiplied by the "PLLMUL" value below
  // then divided by 2.  So, 8MHz*16/2 = 64 MHz
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  // If RCC_PLL_MUL12, then 8MHz / 2 *12 = 48 MHz
  	  // Set to "RCC_PLL_MUL4" to make the system clock go 4x slower!
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  // Turn off all the LEDs to start with.
  HAL_GPIO_WritePin(GPIOE, LD4_Pin|LD3_Pin|LD5_Pin|LD7_Pin
                          |LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure LED GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD7_Pin
                           LD9_Pin LD10_Pin LD8_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD7_Pin
                          |LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);


  /* EXTI interrupt init*/
  // Here we are registering the interrupt functions in the NVIC:
  // Interrupt for one of PA0, PB0, PC0, etc.
  // Make this priority 1 so it is lower priority than the Timer.
  // Also, clear interrupt both at the GPIO level and NVIC level
  // before enabling it.
  __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_0);
  HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  // Recall that Pins 5-9 all share the same interrupt, this is it:
  __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_6);
  HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

// Here, we configure Timer 6 to be incrementing at 2000 Hz and to go off every 2 seconds.
// Note the system clock is 64 MHz in this example.
void Configure_Timer6(void) {
	__HAL_RCC_TIM6_CLK_ENABLE(); 	// Connects TIM6 to clock system

	my_tim6_handle.Instance = TIM6;
	my_tim6_handle.Init.Prescaler = 32000-1; //64MHz/32000 = 2000Hz
	// Subtract 1 to get the division correct per the formula in lecture slides.
	// Recall the maximum value of a 16-bit number is 2^16-1 = 65535
	my_tim6_handle.Init.Period = 4000-1;   // 2000Hz / 4000 = 0.5Hz = 2.0s

	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 0);
		// This is priority 2 so it will not take precedence over the Buttons.
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

	HAL_TIM_Base_Init(&my_tim6_handle);  // actual initialization

    __HAL_TIM_CLEAR_IT(&my_tim6_handle, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&my_tim6_handle, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(&my_tim6_handle, 0);

}

// Here, we configure Timer 7 to be at 16000 Hz, for a
// duration of 1 second.
// Note the system clock is 64 MHz in this example.
void Configure_Timer7(void) {
	__HAL_RCC_TIM7_CLK_ENABLE(); 	// Connects TIM7 to clock system

	my_tim7_handle.Instance = TIM7;
	my_tim7_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	my_tim7_handle.Init.Prescaler = 4000-1; //64MHz/4000 = 16000Hz
	// Subtract 1 to get the division correct per the formula in lecture slides.
	// Recall the maximum value of a 16-bit number is 2^16-1 = 65535
	my_tim7_handle.Init.Period = 16000-1;   // 16000Hz / 16000 = 1Hz = 1.0s
	// This is default so no need to set it:
//	my_tim7_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;


	HAL_NVIC_SetPriority(TIM7_IRQn, 2, 1);
		// This is priority 2 so it not will take precedence over the Buttons.
		// Has a lower sub-priority than Timer6.
	    // (just as an example, it doesn't really matter here)
	HAL_NVIC_EnableIRQ(TIM7_IRQn);

    // Initialize Timer 7
    HAL_TIM_Base_Init(&my_tim7_handle);

    __HAL_TIM_CLEAR_IT(&my_tim7_handle, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&my_tim7_handle, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COUNTER(&my_tim7_handle, 0);

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
  *         where the assert_param error has occurred.
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
