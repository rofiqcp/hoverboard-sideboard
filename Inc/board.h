#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

/*
 * Modul board hanya mengurus hardware dasar sideboard:
 * clock 64 MHz, I2C1 untuk IMU, USART2 untuk serial, dan timestamp mikrodetik.
 */
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

void board_init(void);
uint32_t board_micros(void);

/* Scheduler hardware 100 Hz untuk service estimator/FIFO. TIM2 berjalan
 * independen dari SysTick sehingga periode tidak terkuantisasi 1 ms. */
void board_imu_scheduler_reset(void);
uint32_t board_wait_imu_tick(void);

/* Memulihkan bus I2C yang macet dengan 9 pulsa SCL lalu membuat kondisi STOP. */
void board_i2c_recover(void);

/* Kondisi aman terakhir jika inisialisasi hardware dasar benar-benar gagal. */
void board_panic(void);

/* Minta bootloader aktif setelah software reset. Magic disimpan di backup register. */
void board_reboot_to_bootloader(void);

/* Ambil satu byte RX USART2 dari ring buffer ISR. Return 1 jika ada data. */
int board_uart_rx_pop(uint8_t *out);

/* Kirim frame telemetry tanpa blocking memakai interrupt TXE USART2. */
int board_uart_tx_async(const uint8_t *data, uint16_t len);
int board_uart_tx_busy(void);
void board_uart_tx_wait_idle(uint32_t timeout_us);

#endif
