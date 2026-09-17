#include "stm32f1xx.h"
#include <stdint.h>

/* Bootloader sangat kecil: UART2 921600, framing VESC pendek, flash aplikasi.
 * Clock dibuat 64 MHz dari HSI/2 x16. APB1 = 32 MHz agar baud tinggi stabil. */
#define APP_START       0x08001800UL
#define APP_IMAGE_END   0x0800F800UL
#define APP_META_ADDR   0x0800F800UL
#define EEPROM_ADDR     0x0800FC00UL
#define PAGE_SIZE       1024UL
#define BOOT_WAIT_MS    1500UL
#define CMD_INFO        0xF8U
#define CMD_ERASE       0xF9U
#define CMD_WRITE       0xFAU
#define CMD_GO          0xFBU
#define CMD_VERIFY      0xFCU
#define BOOT_REQUEST_MAGIC 0xB007U
#define APP_MANIFEST_MAGIC 0x41505056UL /* APPV */

static uint32_t ms_counter;

typedef struct {
    uint32_t magic;
    uint32_t length;
    uint32_t length_inv;
    uint16_t crc16;
    uint16_t crc16_inv;
    uint32_t magic_inv;
} AppManifest;

static void backup_domain_enable(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
    PWR->CR |= PWR_CR_DBP;
}

static uint8_t boot_request_active(void)
{
    backup_domain_enable();
    return ((uint16_t)BKP->DR1 == BOOT_REQUEST_MAGIC) ? 1U : 0U;
}

static void boot_request_set(void)
{
    backup_domain_enable();
    BKP->DR1 = BOOT_REQUEST_MAGIC;
}

static void boot_request_clear(void)
{
    backup_domain_enable();
    BKP->DR1 = 0U;
}

static void watchdog_kick(void) { IWDG->KR = 0xAAAAU; }

static void watchdog_init(void)
{
    IWDG->KR = 0x5555U;
    IWDG->PR = 6U;       /* /256 */
    IWDG->RLR = 1562U;   /* ~10 s nominal @ LSI 40 kHz */
    IWDG->KR = 0xAAAAU;
    IWDG->KR = 0xCCCCU;
}


static uint16_t crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0;
    for (uint16_t i=0; i<len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b=0; b<8; b++)
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
    return crc;
}

static void hw_init(void)
{
    /* Samakan clock bootloader dengan aplikasi: HSI 8 MHz / 2 x 16 = 64 MHz.
     * APB1 dibagi 2 menjadi 32 MHz karena batas STM32F103 untuk APB1 adalah 36 MHz. */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) { }

    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;
    RCC->CFGR = RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PLLMULL16;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { }
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2 = alternate push-pull 50 MHz (0xB), PA3 = input pull-up (0x8). */
    GPIOA->CRL &= ~((0xFU << 8) | (0xFU << 12));
    GPIOA->CRL |=  (0xBU << 8) | (0x8U << 12);
    GPIOA->ODR |= GPIO_ODR_ODR3;

    /* PCLK1=32 MHz. BRR=35 (0x23) memberi ~914286 baud, error -0.79% dari 921600. */
    USART2->BRR = 0x23U;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    SysTick->LOAD = 64000U - 1U;
    SysTick->VAL = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

static uint32_t boot_millis(void)
{
    if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) ms_counter++;
    return ms_counter;
}

static int uart_get(uint8_t *out)
{
    uint32_t sr = USART2->SR;
    if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        (void)USART2->DR; /* SR lalu DR membersihkan error STM32F1. */
        return 0;
    }
    if (!(sr & USART_SR_RXNE)) return 0;
    *out = (uint8_t)USART2->DR;
    return 1;
}

static void uart_put(uint8_t b)
{
    while (!(USART2->SR & USART_SR_TXE)) { watchdog_kick(); }
    USART2->DR = b;
}

static void send_packet(const uint8_t *payload, uint8_t len)
{
    uint16_t crc = crc16(payload, len);
    uart_put(2U); uart_put(len);
    for (uint8_t i=0; i<len; i++) uart_put(payload[i]);
    uart_put((uint8_t)(crc >> 8));
    uart_put((uint8_t)crc);
    uart_put(3U);
}

static int flash_wait(void)
{
    while (FLASH->SR & FLASH_SR_BSY) { watchdog_kick(); }
    if (FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)) {
        FLASH->SR = FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
        return 0;
    }
    return 1;
}

static void flash_unlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123UL;
        FLASH->KEYR = 0xCDEF89ABUL;
    }
}

static void flash_lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

static int erase_page_unlocked(uint32_t addr)
{
    if (!flash_wait()) return 0;
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR = addr;
    FLASH->CR |= FLASH_CR_STRT;
    if (!flash_wait()) { FLASH->CR &= ~FLASH_CR_PER; return 0; }
    FLASH->CR &= ~FLASH_CR_PER;
    return 1;
}

static int erase_app(void)
{
    flash_unlock();
    /* Invalidate persistent app marker FIRST. Power loss setelah titik ini tidak
     * boleh pernah dianggap app valid, walaupun vector page sudah sempat tertulis. */
    if (!erase_page_unlocked(APP_META_ADDR)) { flash_lock(); return 0; }
    for (uint32_t a = APP_START; a < APP_IMAGE_END; a += PAGE_SIZE) {
        watchdog_kick();
        if (!erase_page_unlocked(a)) { flash_lock(); return 0; }
    }
    flash_lock();
    return 1;
}

static int program_data(uint32_t addr, const uint8_t *data, uint16_t len)
{
    /* Cek panjang dengan pengurangan supaya addr+len tidak bisa overflow. */
    if (!data || len == 0U || addr < APP_START || addr >= APP_IMAGE_END ||
        (uint32_t)len > (APP_IMAGE_END - addr) || (addr & 1U)) {
        return 0;
    }

    flash_unlock();
    for (uint16_t off = 0; off < len; off += 2U) {
        uint16_t hw = data[off];
        if ((uint16_t)(off + 1U) < len) {
            hw |= (uint16_t)data[off + 1U] << 8;
        } else {
            hw |= 0xFF00U;
        }

        uint16_t current = *(volatile uint16_t *)(addr + off);
        if (current == hw) continue; /* ACK hilang -> resend chunk aman/idempotent. */
        if ((current & hw) != hw) { flash_lock(); return 0; } /* Butuh 0->1: wajib ERASE ulang. */
        if (!flash_wait()) { flash_lock(); return 0; }
        FLASH->CR |= FLASH_CR_PG;
        *(volatile uint16_t *)(addr + off) = hw;
        if (!flash_wait()) { FLASH->CR &= ~FLASH_CR_PG; flash_lock(); return 0; }
        FLASH->CR &= ~FLASH_CR_PG;
        if (*(volatile uint16_t *)(addr + off) != hw) { flash_lock(); return 0; }
    }
    flash_lock();
    return 1;
}

static int image_crc_matches(uint32_t length, uint16_t expected_crc)
{
    if (length == 0U || length > (APP_IMAGE_END - APP_START) || length > 65535U) return 0;
    const uint8_t *image = (const uint8_t *)APP_START;
    return crc16(image, (uint16_t)length) == expected_crc;
}

static int manifest_header_valid(AppManifest *out)
{
    const volatile AppManifest *m = (const volatile AppManifest *)APP_META_ADDR;
    AppManifest v;
    v.magic=m->magic; v.length=m->length; v.length_inv=m->length_inv;
    v.crc16=m->crc16; v.crc16_inv=m->crc16_inv; v.magic_inv=m->magic_inv;
    if (v.magic != APP_MANIFEST_MAGIC || v.magic_inv != ~APP_MANIFEST_MAGIC ||
        v.length == 0U || v.length > (APP_IMAGE_END-APP_START) || v.length > 65535U ||
        v.length_inv != ~v.length || v.crc16_inv != (uint16_t)~v.crc16) return 0;
    if (out) *out=v;
    return 1;
}

static int write_manifest(uint32_t length, uint16_t crc)
{
    AppManifest m;
    m.magic=APP_MANIFEST_MAGIC; m.length=length; m.length_inv=~length;
    m.crc16=crc; m.crc16_inv=(uint16_t)~crc; m.magic_inv=~APP_MANIFEST_MAGIC;

    flash_unlock();
    /* VERIFY boleh diulang: metadata page dibuat fresh setiap commit. */
    if (!erase_page_unlocked(APP_META_ADDR)) { flash_lock(); return 0; }
    const uint8_t *raw=(const uint8_t *)&m;
    for (uint16_t off=0U; off<sizeof(m); off+=2U) {
        watchdog_kick();
        uint16_t hw=(uint16_t)raw[off] | ((uint16_t)raw[off+1U]<<8);
        if (!flash_wait()) { flash_lock(); return 0; }
        FLASH->CR |= FLASH_CR_PG;
        *(volatile uint16_t *)(APP_META_ADDR+off)=hw;
        if (!flash_wait()) { FLASH->CR &= ~FLASH_CR_PG; flash_lock(); return 0; }
        FLASH->CR &= ~FLASH_CR_PG;
        if (*(volatile uint16_t *)(APP_META_ADDR+off)!=hw) { flash_lock(); return 0; }
    }
    flash_lock();
    AppManifest verify;
    return manifest_header_valid(&verify) && verify.length==length && verify.crc16==crc;
}

static int verify_and_commit_app(uint32_t length, uint16_t expected_crc)
{
    if (!image_crc_matches(length,expected_crc)) return 0;
    return write_manifest(length,expected_crc);
}

static int app_valid(void)
{
    uint32_t sp = *(volatile uint32_t *)APP_START;
    uint32_t pc = *(volatile uint32_t *)(APP_START + 4U);
    uint32_t pc_addr = pc & ~1UL;
    AppManifest m;
    if (!(sp >= 0x20000000UL && sp <= 0x20005000UL &&
          pc_addr >= APP_START && pc_addr < APP_IMAGE_END && (pc & 1U))) return 0;
    if (!manifest_header_valid(&m)) return 0;
    return image_crc_matches(m.length,m.crc16);
}

__attribute__((naked, noreturn)) static void jump_raw(uint32_t app_sp, uint32_t app_pc)
{
    /* Fungsi naked wajib: setelah MSP diganti tidak boleh ada POP/akses stack bootloader lagi. */
    __asm volatile (
        "msr msp, r0\n"
        "cpsie i\n"
        "bx r1\n"
    );
}

static void jump_app(void)
{
    if (!app_valid()) return;

    uint32_t app_sp = *(volatile uint32_t *)APP_START;
    uint32_t app_pc = *(volatile uint32_t *)(APP_START + 4U);

    SysTick->CTRL = 0U;
    USART2->CR1 = 0U;
    __disable_irq();

    /* Bersihkan interrupt pending agar aplikasi mulai seperti setelah reset normal. */
    for (uint8_t i = 0; i < 8U; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }
    SCB->VTOR = APP_START;
    __DSB();
    __ISB();
    jump_raw(app_sp, app_pc);
}

static uint32_t get_u32_be(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static void put_u32_be(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);  p[3] = (uint8_t)v;
}

static uint8_t process_payload(const uint8_t *p, uint8_t len)
{
    uint8_t reply[16];
    if (!p || len == 0U) return 0U;

    if (p[0] == CMD_INFO) {
        boot_request_set();
        reply[0] = CMD_INFO; reply[1] = 0U; reply[2] = 3U;
        put_u32_be(&reply[3], APP_START);
        put_u32_be(&reply[7], APP_IMAGE_END);
        reply[11] = (uint8_t)(PAGE_SIZE >> 8);
        reply[12] = (uint8_t)PAGE_SIZE;
        reply[13] = 128U;
        reply[14] = app_valid() ? 1U : 0U;
        send_packet(reply, 15U);
        return 1U;
    }

    if (p[0] == CMD_ERASE && len == 1U) {
        boot_request_set();
        reply[0] = CMD_ERASE;
        reply[1] = erase_app() ? 0U : 1U;
        send_packet(reply, 2U);
        return 1U;
    }

    if (p[0] == CMD_WRITE && len >= 6U) {
        boot_request_set();
        uint32_t addr = get_u32_be(&p[1]);
        uint16_t data_len = (uint16_t)len - 5U;
        reply[0] = CMD_WRITE;
        reply[1] = program_data(addr, &p[5], data_len) ? 0U : 1U;
        put_u32_be(&reply[2], addr);
        send_packet(reply, 6U);
        return 1U;
    }

    if (p[0] == CMD_VERIFY && len == 7U) {
        boot_request_set();
        uint32_t image_len = get_u32_be(&p[1]);
        uint16_t expected_crc = ((uint16_t)p[5] << 8) | p[6];
        reply[0] = CMD_VERIFY;
        reply[1] = verify_and_commit_app(image_len, expected_crc) ? 0U : 1U;
        send_packet(reply, 2U);
        return 1U;
    }

    if (p[0] == CMD_GO && len == 1U) {
        boot_request_set();
        uint8_t valid = app_valid() ? 1U : 0U;
        reply[0] = CMD_GO; reply[1] = valid ? 0U : 1U;
        send_packet(reply, 2U);
        if (valid) boot_request_clear();
        for (volatile uint32_t i = 0; i < 40000U; i++) { __NOP(); }
        jump_app();
        return 1U;
    }

    reply[0] = p[0]; reply[1] = 2U;
    send_packet(reply, 2U);
    return 1U;
}

typedef struct {
    uint8_t state;
    uint8_t len;
    uint8_t index;
    uint8_t payload[192];
    uint16_t crc_rx;
    uint32_t last_byte_ms;
} PacketRx;

static uint8_t packet_feed(PacketRx *rx, uint8_t b)
{
    switch (rx->state) {
    case 0: /* Menunggu byte pembuka VESC untuk paket pendek. */
        if (b == 2U) rx->state = 1U;
        break;
    case 1:
        if (b == 0U || b > sizeof(rx->payload)) {
            rx->state = (b == 2U) ? 1U : 0U;
        } else {
            rx->len = b; rx->index = 0U; rx->state = 2U;
        }
        break;
    case 2:
        rx->payload[rx->index++] = b;
        if (rx->index >= rx->len) rx->state = 3U;
        break;
    case 3:
        rx->crc_rx = (uint16_t)b << 8; rx->state = 4U;
        break;
    case 4:
        rx->crc_rx |= b; rx->state = 5U;
        break;
    case 5: {
        uint8_t ok = (b == 3U && rx->crc_rx == crc16(rx->payload, rx->len)) ? 1U : 0U;
        rx->state = (!ok && b == 2U) ? 1U : 0U;
        if (ok) return process_payload(rx->payload, rx->len);
        break;
    }
    default:
        rx->state = 0U;
        break;
    }
    return 0U;
}

int main(void)
{
    /* Simpan alasan reset sebelum flag dibersihkan. Watchdog aplikasi mendapat
     * recovery window 5 s; F1/update aktif memakai backup latch dan stay tanpa batas. */
    uint8_t watchdog_reset = (RCC->CSR & RCC_CSR_IWDGRSTF) ? 1U : 0U;
    RCC->CSR |= RCC_CSR_RMVF;
    hw_init();
    watchdog_init();
    PacketRx rx = {0};
    uint8_t stay_in_boot = boot_request_active();
    uint32_t start_ms = boot_millis();
    uint32_t auto_boot_wait = watchdog_reset ? 5000U : BOOT_WAIT_MS;

    for (;;) {
        watchdog_kick();
        uint32_t now_ms = boot_millis();
        if (rx.state != 0U && (uint32_t)(now_ms - rx.last_byte_ms) > 25U) rx.state = 0U;

        uint8_t b;
        if (uart_get(&b)) {
            now_ms = boot_millis();
            if (rx.state != 0U && (uint32_t)(now_ms - rx.last_byte_ms) > 25U) rx.state = 0U;
            rx.last_byte_ms = now_ms;
            if (packet_feed(&rx, b)) stay_in_boot = 1U;
        }

        /* Valid app normal: boot setelah window. Reset watchdog app: beri host 5 s
         * untuk INFO/F1 sebelum auto-retry. App invalid atau latch update: stay. */
        if (!stay_in_boot && (uint32_t)(boot_millis() - start_ms) >= auto_boot_wait) {
            if (app_valid()) jump_app();
            stay_in_boot = 1U; /* Invalid manifest/image: recovery mode forever. */
        }
    }
}

/* Handler minimal. Bootloader tidak membutuhkan interrupt selain fault default. */
void SysTick_Handler(void) { }
void NMI_Handler(void) { }
void HardFault_Handler(void) { for (;;) { } }
void MemManage_Handler(void) { for (;;) { } }
void BusFault_Handler(void) { for (;;) { } }
void UsageFault_Handler(void) { for (;;) { } }
void SVC_Handler(void) { }
void DebugMon_Handler(void) { }
void PendSV_Handler(void) { }
