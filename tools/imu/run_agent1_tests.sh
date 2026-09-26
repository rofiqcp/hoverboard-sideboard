#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."

tmp="$(mktemp -d "${TMPDIR:-/tmp}/agent1_imu.XXXXXX")"
trap 'rm -rf "$tmp"' EXIT

cat > "$tmp/stm32f1xx_hal.h" <<'STUB'
#ifndef STM32F1XX_HAL_H
#define STM32F1XX_HAL_H
#include <stdint.h>
typedef struct { uint32_t dummy; } I2C_HandleTypeDef;
typedef struct { uint32_t dummy; } UART_HandleTypeDef;
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
#define I2C_MEMADD_SIZE_8BIT 1U
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *, uint16_t, uint16_t,
                                    uint16_t, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *, uint16_t, uint16_t,
                                   uint16_t, uint8_t *, uint16_t, uint32_t);
void HAL_Delay(uint32_t ms);
#endif
STUB
out="$tmp/test_agent1_imu_health"
gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$tmp" -IInc \
  tests/test_agent1_imu_health.c Src/imu_mpu6xxx.c -o "$out"
"$out"
