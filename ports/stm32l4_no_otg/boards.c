/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ha Thach for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "board_api.h"
#include "stm32l433xx.h"
#include "stm32l4xx.h"
#include "tinycrypt/sha256.h"
#include "tinyp256.h"
#include "tusb.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+

#define STM32_UUID ((volatile uint32_t *)UID_BASE)

UART_HandleTypeDef UartHandle;

void board_init(void) {
  clock_init();
  SystemCoreClockUpdate();

  // disable systick
  board_timer_stop();

  // TODO enable only used GPIO clock
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

#if (defined(BUTTON_PIN) || defined(LED_PIN))
  GPIO_InitTypeDef GPIO_InitStruct;
#endif

#ifdef BUTTON_PIN
  GPIO_InitStruct.Pin = BUTTON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);
#endif

#ifdef LED_PIN
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  board_led_write(0);
#endif

#if NEOPIXEL_NUMBER
  GPIO_InitStruct.Pin = NEOPIXEL_PIN;
  GPIO_InitStruct.Mode = NEOPIXEL_PIN_MODE;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(NEOPIXEL_PORT, &GPIO_InitStruct);
#endif

#if defined(UART_DEV) && CFG_TUSB_DEBUG
  UART_CLOCK_ENABLE();

  GPIO_InitStruct.Pin = UART_TX_PIN | UART_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = UART_GPIO_AF;
  HAL_GPIO_Init(UART_GPIO_PORT, &GPIO_InitStruct);

  UartHandle.Instance = UART_DEV;
  UartHandle.Init.BaudRate = 115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits = UART_STOPBITS_1;
  UartHandle.Init.Parity = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode = UART_MODE_TX_RX;
  HAL_UART_Init(&UartHandle);
#endif
}

void board_dfu_init(void) {
#ifdef PWR_CR2_USV
  HAL_PWREx_EnableVddUSB();
#endif

  GPIO_InitTypeDef GPIO_InitStruct;

  // USB Pin Init
  // PA9- VUSB, PA10- ID, PA11- DM, PA12- DP

  /* Configure DM DP Pins */
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = GPIO_AF10_USB_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure VBUS Pin */
#ifndef USB_NO_VBUS_PIN
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif

#ifndef USB_NO_USB_ID_PIN
  /* This for ID line debug */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif

  // Enable USB OTG clock
  //__HAL_RCC_USB_OTG_FS_CLK_ENABLE();
  __HAL_RCC_USB_CLK_ENABLE();

#if 0
#ifdef USB_NO_VBUS_PIN
  // Disable VBUS sense
  USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
#else
  // Enable VBUS sense (B device) via pin PA9
  USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_NOVBUSSENS;
  USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_VBUSBSEN;
#endif
#endif

  // Deactivate VBUS Sensing B
  // USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBDEN;

  // #ifdef USB_NO_USB_ID_PIN
  // USB_OTG_FS->GUSBCFG &= ~USB_OTG_GUSBCFG_FHMOD;
  // USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
  // #endif

  // // B-peripheral session valid override enable
  // USB_OTG_FS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
  // USB_OTG_FS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
}

void board_reset(void) { NVIC_SystemReset(); }

void board_dfu_complete(void) {
  // todo - eject USB drive for smooth UX

  NVIC_SystemReset();
}

bool board_app_sign_valid(void);

bool board_app_valid(void) {
  volatile uint32_t const *app_vector =
      (volatile uint32_t const *)BOARD_FLASH_APP_START;

  if (board_app_sign_valid() != true)
    return false;

  // 1st word is stack pointer (should be in SRAM region)
  if (app_vector[0] < BOARD_STACK_APP_START ||
      app_vector[0] > BOARD_STACK_APP_END) {
    return false;
  }

  // 2nd word is App entry point (reset)
  if (app_vector[1] < BOARD_FLASH_APP_START ||
      app_vector[1] > BOARD_FLASH_APP_START + BOARD_FLASH_SIZE) {
    return false;
  }

  return true;
}

#define ECDSA_SIGN_LEN (64)

#define TINYUF2_CHECK_SHA256
static const uint32_t marker_cmp = 0xA5A55A5A;

typedef struct __attribute__((packed)) {
  uint32_t marker; // 0xA5A55A5A
  uint32_t fw_len;
  uint8_t fw_signature[ECDSA_SIGN_LEN];
  uint32_t fw_version[2];          // two bytes
  uint32_t date_of_compilation[6]; // year YYYY, month MM, day DD, hour HH, min
                                   // MM, sec SS
} fw_control_sector_t;

static struct {
  uint8_t iv[16];
  uint8_t key[16];
} aes_enc_keys = {
    .iv = {0xd9, 0x59, 0x56, 0xbd, 0xdf, 0xc4, 0x59, 0x8c, 0x17, 0x1b, 0xd1,
           0xe0, 0xe0, 0x9e, 0x75, 0x0b},
    .key = {0xf5, 0xfe, 0x2d, 0x45, 0x07, 0x09, 0xd8, 0xd5, 0x98, 0x05, 0x8a,
            0x6d, 0x4f, 0xc6, 0x2f, 0xae},
};

/* This is space for bootloader signature for FW and bootloader to selfcheck to
 * check */
static const uint8_t BOOTLOADER_SIGNATURE[]
    __attribute__((section(".BootSignSection"))) __attribute__((used)) = {
        0xBB, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xAB};

static const uint8_t public_key[] __attribute__((section(".PubKeySection"))) = {
    0x69, 0x2d, 0x32, 0x6f, 0x1c, 0x40, 0xc6, 0x8a, 0x6f, 0x90, 0x65,
    0xa2, 0x04, 0x3b, 0x6d, 0xa8, 0xc5, 0xf3, 0xf7, 0x61, 0x9f, 0xd5,
    0x90, 0xa2, 0x79, 0x36, 0xdf, 0x27, 0x8e, 0x3d, 0x0b, 0x8e, 0x6d,
    0xb4, 0x95, 0x82, 0xf2, 0x98, 0xe7, 0xa3, 0xe7, 0x18, 0xde, 0xed,
    0x23, 0x0c, 0x9c, 0x4d, 0xf7, 0x03, 0xe5, 0x04, 0x6a, 0xf6, 0x82,
    0x5f, 0x71, 0x3a, 0xa4, 0x6c, 0x3f, 0xe6, 0x36, 0x60};

static uint8_t sha256_digest[32];
struct tc_sha256_state_struct s;

bool board_app_sign_valid(void) {
  volatile fw_control_sector_t const *fw_ctrl =
      (volatile fw_control_sector_t const *)BOARD_FLASH_CONTROL_SECTOR;
  (void)BOOTLOADER_SIGNATURE;

#if defined(TINYUF2_CHECK_SHA256)
  (void)fw_ctrl;
  uint32_t res = fw_ctrl->marker ^ marker_cmp;
  if (res != 0) {
    return false;
  }
  if (fw_ctrl->fw_len >
      (BOARD_FLASH_SIZE - (BOARD_FLASH_APP_START & 0x00FFFFFF))) {
    return false;
  }

  (void)tc_sha256_init(&s);
  tc_sha256_update(&s, (const uint8_t *)BOARD_FLASH_APP_START, fw_ctrl->fw_len);
  (void)tc_sha256_final(sha256_digest, &s);

  tinyp256_t result = tinyp256_verify(public_key, sizeof(public_key),
                                      sha256_digest, sizeof(sha256_digest),
                                      (uint8_t *)fw_ctrl->fw_signature, 64);
  if (result == TINYP256_OK) {
    return true;
  }

#else
  (void)fw_ctrl;
#endif

  return false;
}

void board_app_jump(void) {
  volatile uint32_t const *app_vector =
      (volatile uint32_t const *)BOARD_FLASH_APP_START;

#ifdef BUTTON_PIN
  HAL_GPIO_DeInit(BUTTON_PORT, BUTTON_PIN);
#endif

#ifdef LED_PIN
  HAL_GPIO_DeInit(LED_PORT, LED_PIN);
#endif

#if NEOPIXEL_NUMBER
  HAL_GPIO_DeInit(NEOPIXEL_PORT, NEOPIXEL_PIN);
#endif

#if defined(UART_DEV) && CFG_TUSB_DEBUG
  HAL_UART_DeInit(&UartHandle);
  HAL_GPIO_DeInit(UART_GPIO_PORT, UART_TX_PIN | UART_RX_PIN);
  UART_CLOCK_DISABLE();
#endif

  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();

  HAL_RCC_DeInit();

  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;

  /* Disable all interrupts */
  RCC->CIER = 0x00000000U;

  // TODO protect bootloader region

  /* switch exception handlers to the application */
  SCB->VTOR = (uint32_t)BOARD_FLASH_APP_START;

  // Set stack pointer
  __set_MSP(app_vector[0]);

  // Jump to Application Entry
  asm("bx %0" ::"r"(app_vector[1]));
}

uint8_t board_usb_get_serial(uint8_t serial_id[16]) {
  uint8_t const len = 12;
  uint32_t *serial_id32 = (uint32_t *)(uintptr_t)serial_id;

  serial_id32[0] = STM32_UUID[0];
  serial_id32[1] = STM32_UUID[1];
  serial_id32[2] = STM32_UUID[2];

  return len;
}

//--------------------------------------------------------------------+
// LED pattern
//--------------------------------------------------------------------+

void board_led_write(uint32_t state) {
#ifdef LED_PIN
  HAL_GPIO_WritePin(LED_PORT, LED_PIN,
                    state ? LED_STATE_ON : (1 - LED_STATE_ON));
#else
  UNUSED(state);
#endif
}

#if NEOPIXEL_NUMBER
#define MAGIC_800_INT 900000  // ~1.11 us -> 1.2  field
#define MAGIC_800_T0H 2800000 // ~0.36 us -> 0.44 field
#define MAGIC_800_T1H 1350000 // ~0.74 us -> 0.84 field

static inline uint8_t apply_percentage(uint8_t brightness) {
  return (uint8_t)((brightness * NEOPIXEL_BRIGHTNESS) >> 8);
}

void board_rgb_write(uint8_t const rgb[]) {
  // assumes 800_000Hz frequency
  // Theoretical values here are 800_000 -> 1.25us, 2500000->0.4us,
  // 1250000->0.8us
  uint32_t const sys_freq = HAL_RCC_GetSysClockFreq();
  uint32_t const interval = sys_freq / MAGIC_800_INT;
  uint32_t const t0 = sys_freq / MAGIC_800_T0H;
  uint32_t const t1 = sys_freq / MAGIC_800_T1H;

  // neopixel color order is GRB
  uint8_t const colors[3] = {apply_percentage(rgb[1]), apply_percentage(rgb[0]),
                             apply_percentage(rgb[2])};

  __disable_irq();
  uint32_t start;
  uint32_t cyc;

  // Enable DWT in debug core. Useable when interrupts disabled, as opposed to
  // Systick->VAL
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;

  for (uint32_t i = 0; i < NEOPIXEL_NUMBER; i++) {
    uint8_t const *color_pointer = colors;
    uint8_t const *const color_pointer_end = color_pointer + 3;
    uint8_t color = *color_pointer++;
    uint8_t color_mask = 0x80;

    while (true) {
      cyc = (color & color_mask) ? t1 : t0;
      start = DWT->CYCCNT;

      HAL_GPIO_WritePin(NEOPIXEL_PORT, NEOPIXEL_PIN, 1);
      while ((DWT->CYCCNT - start) < cyc)
        ;

      HAL_GPIO_WritePin(NEOPIXEL_PORT, NEOPIXEL_PIN, 0);
      while ((DWT->CYCCNT - start) < interval)
        ;

      if (!(color_mask >>= 1)) {
        if (color_pointer >= color_pointer_end) {
          break;
        }
        color = *color_pointer++;
        color_mask = 0x80;
      }
    }
  }

  __enable_irq();
}

#else

void board_rgb_write(uint8_t const rgb[]) { (void)rgb; }

#endif

//--------------------------------------------------------------------+
// Timer
//--------------------------------------------------------------------+

void board_timer_start(uint32_t ms) {
  SysTick_Config((SystemCoreClock / 1000) * ms);
}

void board_timer_stop(void) { SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; }

void SysTick_Handler(void) { board_timer_handler(); }

int board_uart_write(void const *buf, int len) {
#if defined(UART_DEV) && CFG_TUSB_DEBUG
  HAL_UART_Transmit(&UartHandle, (uint8_t *)buf, len, 0xffff);
  return len;
#else
  (void)buf;
  (void)len;
  (void)UartHandle;
  return 0;
#endif
}

#ifndef TINYUF2_SELF_UPDATE

// Forward USB interrupt events to TinyUSB IRQ Handler
void USB_IRQHandler(void) { tud_int_handler(0); }

#endif

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void) {}
