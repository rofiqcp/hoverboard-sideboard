#include "board.h"
#include "app_config.h"
#include <string.h>

/*
 * Semua komentar pada modul ini memakai Bahasa Indonesia agar alur hardware
 * mudah ditelusuri saat troubleshooting langsung di board.
 */
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

#define UART_RX_RING_SIZE 64U
static volatile uint8_t uart_rx_ring[UART_RX_RING_SIZE];
static volatile uint8_t uart_rx_head = 0U;
static volatile uint8_t uart_rx_tail = 0U;

#define UART_TX_BUFFER_SIZE 128U
static uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];
static volatile uint16_t uart_tx_len = 0U;
static volatile uint16_t uart_tx_index = 0U;
static volatile uint8_t uart_tx_busy_flag = 0U;

/* TIM2 dipakai sebagai scheduler IMU 100 Hz. Counter pending membuat tick yang
 * datang ketika CPU masih bekerja tetap terlihat, tanpa menjalankan ESKF di ISR. */
static volatile uint32_t imu_tick_pending = 0U;
static volatile uint32_t imu_tick_total = 0U;

static void clock_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    /* HSI 8 MHz / 2 x 16 = SYSCLK 64 MHz. */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    osc.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) board_panic();

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) board_panic();
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *h)
{
    if (h->Instance != I2C1) return;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
}

void HAL_UART_MspInit(UART_HandleTypeDef *h)
{
    if (h->Instance != USART2) return;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void i2c_init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000U;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0U;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0U;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) board_panic();
}

static void uart_init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = APP_UART_BAUD;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) board_panic();

    /* RX interrupt menjaga command servis tidak hilang saat TX telemetry sedang blocking. */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    HAL_NVIC_SetPriority(USART2_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}


static void imu_scheduler_init(void)
{
    /* APB1=32 MHz dengan prescaler /2; clock timer APB1 menjadi 64 MHz.
     * PSC=63 -> counter 1 MHz, ARR=9999 -> update tepat 100 Hz. */
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->CR1 = 0U;
    TIM2->PSC = 63U;
    TIM2->ARR = 9999U;
    TIM2->CNT = 0U;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
    TIM2->DIER = TIM_DIER_UIE;
    imu_tick_pending = 0U;
    imu_tick_total = 0U;
    HAL_NVIC_SetPriority(TIM2_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 = TIM_CR1_CEN;
}

static void i2c_recovery_delay(void)
{
    /* Jeda beberapa mikrodetik; presisi tidak kritis karena hanya dipakai saat recovery. */
    for (volatile uint32_t i = 0; i < 160U; i++) __NOP();
}

void board_i2c_recover(void)
{
    /* Reset peripheral, lalu lepaskan slave yang mungkin menahan SDA dengan 9 clock. */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    i2c_recovery_delay();

    for (uint8_t i = 0; i < 9U; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        i2c_recovery_delay();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        i2c_recovery_delay();
    }

    /* STOP manual: SDA low -> SCL high -> SDA high. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    i2c_recovery_delay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    i2c_recovery_delay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    i2c_recovery_delay();

    hi2c1.State = HAL_I2C_STATE_RESET;
    hi2c1.Lock = HAL_UNLOCKED;
    i2c_init();
}

void board_init(void)
{
    /* Aplikasi berada setelah bootloader, jadi vector interrupt harus dipindahkan. */
    SCB->VTOR = APP_FLASH_START;
    __DSB();
    __ISB();

    HAL_Init();
    clock_init();
    i2c_init();
    uart_init();
    imu_scheduler_init();

    /* DWT dipakai sebagai timer mikrodetik tanpa interrupt tambahan. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t board_micros(void)
{
    /*
     * Jangan membagi CYCCNT langsung. CYCCNT 32-bit @64 MHz wrap setiap
     * sekitar 67 detik. Dengan mengakumulasi DELTA unsigned, wrap hardware
     * menjadi transparan dan timestamp mikrodetik baru wrap alami ~71,6 menit.
     */
    static uint32_t last_cycles = 0U;
    static uint32_t micros_accum = 0U;
    static uint32_t cycle_remainder = 0U;

    /* SYSCLK firmware ini fixed 64 MHz: 64 cycle = 1 mikrodetik.
     * Shift/mask menghindari operasi divide/modulo di jalur scheduler. */
    const uint32_t cycles_per_us = 64U;
    const uint32_t cycle_mask = cycles_per_us - 1U;

    uint32_t now_cycles = DWT->CYCCNT;
    uint32_t delta_cycles = now_cycles - last_cycles;
    last_cycles = now_cycles;

    uint32_t whole_us = delta_cycles >> 6;
    uint32_t rem_total = cycle_remainder + (delta_cycles & cycle_mask);

    micros_accum += whole_us + (rem_total >> 6);
    cycle_remainder = rem_total & cycle_mask;
    return micros_accum;
}

void board_imu_scheduler_reset(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    imu_tick_pending = 0U;
    TIM2->CNT = 0U;
    TIM2->SR = 0U;
    if (!primask) __enable_irq();
}

uint32_t board_wait_imu_tick(void)
{
    while (imu_tick_pending == 0U) {
        __WFI();
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t pending = imu_tick_pending;
    imu_tick_pending = 0U;
    if (!primask) __enable_irq();
    return pending;
}

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        if (imu_tick_pending < 0xFFFFFFFFUL) imu_tick_pending++;
        imu_tick_total++;
    }
}

int board_uart_rx_pop(uint8_t *out)
{
    if (!out) return 0;
    uint8_t tail = uart_rx_tail;
    if (tail == uart_rx_head) return 0;
    *out = uart_rx_ring[tail];
    uart_rx_tail = (uint8_t)((tail + 1U) % UART_RX_RING_SIZE);
    return 1;
}

int board_uart_tx_async(const uint8_t *data, uint16_t len)
{
    if (!data || len == 0U || len > UART_TX_BUFFER_SIZE || uart_tx_busy_flag) {
        return 0;
    }

    memcpy(uart_tx_buffer, data, len);
    uart_tx_len = len;
    uart_tx_index = 0U;
    uart_tx_busy_flag = 1U;
    SET_BIT(USART2->CR1, USART_CR1_TXEIE);
    return 1;
}

int board_uart_tx_busy(void)
{
    return uart_tx_busy_flag != 0U;
}

void board_uart_tx_wait_idle(uint32_t timeout_us)
{
    uint32_t start = board_micros();
    while (uart_tx_busy_flag) {
        if ((uint32_t)(board_micros() - start) >= timeout_us) {
            break;
        }
        __WFI();
    }
}

void USART2_IRQHandler(void)
{
    uint32_t sr = USART2->SR;

    if (sr & USART_SR_RXNE) {
        uint8_t b = (uint8_t)USART2->DR;
        uint8_t head = uart_rx_head;
        uint8_t next = (uint8_t)((head + 1U) % UART_RX_RING_SIZE);
        if (next != uart_rx_tail) {
            uart_rx_ring[head] = b;
            uart_rx_head = next;
        }
    } else if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        /* Membaca DR setelah SR membersihkan flag error STM32F1. */
        (void)USART2->DR;
    }

    if ((USART2->CR1 & USART_CR1_TXEIE) && (USART2->SR & USART_SR_TXE)) {
        if (uart_tx_index < uart_tx_len) {
            USART2->DR = uart_tx_buffer[uart_tx_index++];
        }
        if (uart_tx_index >= uart_tx_len) {
            CLEAR_BIT(USART2->CR1, USART_CR1_TXEIE);
            uart_tx_busy_flag = 0U;
        }
    }
}

void board_reboot_to_bootloader(void)
{
    /* Backup register bertahan saat NVIC_SystemReset selama board tetap diberi daya. */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
    PWR->CR |= PWR_CR_DBP;
    BKP->DR1 = BOOTLOADER_REQUEST_MAGIC;
    __DSB();
    __ISB();
    NVIC_SystemReset();
    for (;;) { }
}

void board_panic(void)
{
    /* Fail-safe recovery: jangan hard-lock aplikasi. Jika init hardware atau
     * runtime masuk kondisi fatal, paksa software reset ke bootloader dan
     * set magic agar bootloader tetap aktif tanpa timeout. Bootloader memakai
     * register langsung sehingga tidak bergantung pada HAL aplikasi. */
    board_reboot_to_bootloader();
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}
