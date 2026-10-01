/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (full flash library demonstration)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "flash_stm32f103.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_64TH_PAGE_START_ADDR 	0x0800FC00UL
#define FLASH_64TH_PAGE_END_ADDR 	0x0800FFFFUL
#define FLASH_63RD_PAGE_START_ADDR 	0x0800F800UL
#define FLASH_PAGE_SIZE_BYTES		1024UL

/* main scratch page used by every test */
#define TEST_BASE					FLASH_64TH_PAGE_START_ADDR
/* second page, used only to prove that an erase touches exactly one page.
 * It is skipped automatically if it is not blank (i.e. firmware lives there). */
#define TEST_PAGE_B					FLASH_63RD_PAGE_START_ADDR
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char uart_rx_buf[128];
char uart_tx_buf[128];
volatile uint8_t rx_received = 0;
volatile uint16_t rx_len = 0;

char coms_processing_buf[128];

static uint32_t tests_passed = 0;
static uint32_t tests_failed = 0;
static uint32_t tests_skipped = 0;

/* one full flash page of RAM, viewable as bytes / halfwords / words */
static union {
  uint8_t  b[1024];
  uint16_t h[512];
  uint32_t w[256];
} big;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void uart_send(const char *fmt, ...){
  va_list args;
  va_start(args, fmt);
  vsnprintf(uart_tx_buf, sizeof(uart_tx_buf), fmt, args);
  va_end(args);
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_tx_buf, strlen(uart_tx_buf), 1000);
}

/* ---------------------------------------------------------------------------
 * test helpers
 * ------------------------------------------------------------------------ */
static void check(const char *name, int cond){
  if (cond) tests_passed++;
  else      tests_failed++;
  uart_send("[%s] %s\r\n", cond ? "PASS" : "FAIL", name);
}

static void section(const char *title){
  uart_send("\r\n--- %s ---\r\n", title);
}

static int flash_locked(void){
  return (FLASH_CR & FLASH_CR_LOCK) != 0;
}

static int flash_error_flags_clear(void){
  return (FLASH_SR & (FLASH_SR_WRPRTERR | FLASH_SR_PGERR)) == 0;
}

/* returns 1 if the whole page reads 0xFF */
static int page_is_erased(uint32_t page_addr){
  uint8_t tmp[32];
  for (uint32_t off = 0; off < FLASH_PAGE_SIZE_BYTES; off += sizeof(tmp)){
    Flash_ReadByte_Buf(page_addr + off, tmp, sizeof(tmp));
    for (uint32_t j = 0; j < sizeof(tmp); j++)
      if (tmp[j] != 0xFF) return 0;
  }
  return 1;
}

/* compares flash against a RAM buffer, 32 bytes at a time */
static int verify_region(uint32_t addr, const uint8_t *expected, uint32_t len){
  uint8_t tmp[32];
  for (uint32_t off = 0; off < len; off += sizeof(tmp)){
    uint32_t n = (len - off < sizeof(tmp)) ? (len - off) : sizeof(tmp);
    Flash_ReadByte_Buf(addr + off, tmp, n);
    if (memcmp(tmp, expected + off, n) != 0) return 0;
  }
  return 1;
}

/* erase the scratch page and make sure it is really blank */
static void fresh_page(void){
  int ok = Flash_ErasePage(TEST_BASE) && page_is_erased(TEST_BASE);
  check("fresh page: erase + blank check", ok);
}

static void hex_dump(uint32_t addr, uint32_t len){
  uint8_t row[16];
  for (uint32_t off = 0; off < len; off += 16){
    Flash_ReadByte_Buf(addr + off, row, 16);
    uart_send("%08lX: %02X %02X %02X %02X %02X %02X %02X %02X "
              "%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
              (unsigned long)(addr + off),
              row[0], row[1], row[2],  row[3],  row[4],  row[5],  row[6],  row[7],
              row[8], row[9], row[10], row[11], row[12], row[13], row[14], row[15]);
  }
}

/* ---------------------------------------------------------------------------
 * 1. Flash_ErasePage + the three single-value read functions
 * ------------------------------------------------------------------------ */
static void test_erase_and_read(void){
  section("1. Flash_ErasePage, Flash_ReadByte/HlfWord/Word");

  check("ErasePage returns OK", Flash_ErasePage(TEST_BASE));
  check("page reads blank after erase", page_is_erased(TEST_BASE));
  check("ReadByte on blank flash == 0xFF",
        Flash_ReadByte(TEST_BASE) == 0xFF);
  check("ReadHlfWord on blank flash == 0xFFFF",
        Flash_ReadHlfWord(TEST_BASE) == 0xFFFF);
  check("ReadWord on blank flash == 0xFFFFFFFF",
        Flash_ReadWord(TEST_BASE) == 0xFFFFFFFFUL);
  check("erasing an already-blank page is OK",
        Flash_ErasePage(TEST_BASE) && page_is_erased(TEST_BASE));
  check("flash is re-locked after erase", flash_locked());
}

/* ---------------------------------------------------------------------------
 * 2. Flash_WriteWord / WriteHlfWord / WriteByte (single value)
 * ------------------------------------------------------------------------ */
static void test_single_writes(void){
  uint32_t ok;

  section("2. Flash_WriteWord, WriteHlfWord, WriteByte");
  fresh_page();

  /* --- WriteWord --- */
  ok = Flash_WriteWord(TEST_BASE + 0x00, 0x04030201UL);
  check("WriteWord returns OK", ok);
  check("WriteWord: ReadWord matches",
        Flash_ReadWord(TEST_BASE + 0x00) == 0x04030201UL);
  check("WriteWord: ReadHlfWord low/high match",
        Flash_ReadHlfWord(TEST_BASE + 0x00) == 0x0201 &&
        Flash_ReadHlfWord(TEST_BASE + 0x02) == 0x0403);
  check("WriteWord: bytes are little-endian",
        Flash_ReadByte(TEST_BASE + 0x00) == 0x01 &&
        Flash_ReadByte(TEST_BASE + 0x01) == 0x02 &&
        Flash_ReadByte(TEST_BASE + 0x02) == 0x03 &&
        Flash_ReadByte(TEST_BASE + 0x03) == 0x04);

  /* --- WriteHlfWord --- */
  ok = Flash_WriteHlfWord(TEST_BASE + 0x10, 0xBEEF);
  check("WriteHlfWord returns OK", ok);
  check("WriteHlfWord: ReadHlfWord matches",
        Flash_ReadHlfWord(TEST_BASE + 0x10) == 0xBEEF);
  check("WriteHlfWord: bytes are little-endian",
        Flash_ReadByte(TEST_BASE + 0x10) == 0xEF &&
        Flash_ReadByte(TEST_BASE + 0x11) == 0xBE);
  check("WriteHlfWord: neighbouring halfword untouched",
        Flash_ReadHlfWord(TEST_BASE + 0x12) == 0xFFFF);

  /* --- WriteByte, even address (low half, high half padded 0xFF) --- */
  ok = Flash_WriteByte(TEST_BASE + 0x20, 0x5A);
  check("WriteByte (even addr) returns OK", ok);
  check("WriteByte (even addr): value + padding",
        Flash_ReadByte(TEST_BASE + 0x20) == 0x5A &&
        Flash_ReadByte(TEST_BASE + 0x21) == 0xFF);

  /* --- WriteByte, odd address (high half, low half padded 0xFF) --- */
  ok = Flash_WriteByte(TEST_BASE + 0x33, 0xA5);
  check("WriteByte (odd addr) returns OK", ok);
  check("WriteByte (odd addr): value + padding",
        Flash_ReadByte(TEST_BASE + 0x33) == 0xA5 &&
        Flash_ReadByte(TEST_BASE + 0x32) == 0xFF);

  check("flash is re-locked after write APIs", flash_locked());

  /* --- negative tests: programming over non-blank flash must fail --- */
  ok = Flash_WriteHlfWord(TEST_BASE + 0x10, 0x1234);
  check("WriteHlfWord over programmed halfword fails", ok == 0);
  check("  ...old halfword intact", Flash_ReadHlfWord(TEST_BASE + 0x10) == 0xBEEF);

  ok = Flash_WriteWord(TEST_BASE + 0x00, 0x12345678UL);
  check("WriteWord over programmed word fails", ok == 0);
  check("  ...old word intact", Flash_ReadWord(TEST_BASE + 0x00) == 0x04030201UL);

  ok = Flash_WriteByte(TEST_BASE + 0x20, 0x11);
  check("WriteByte over programmed byte fails", ok == 0);
  check("  ...old byte intact", Flash_ReadByte(TEST_BASE + 0x20) == 0x5A);

  check("error flags cleared after failed writes", flash_error_flags_clear());
  check("flash is re-locked after failed writes", flash_locked());

  /* --- still usable afterwards --- */
  ok = Flash_WriteHlfWord(TEST_BASE + 0x40, 0x600D);
  check("write after failures still works",
        ok && Flash_ReadHlfWord(TEST_BASE + 0x40) == 0x600D);
}

/* ---------------------------------------------------------------------------
 * 3. Buffer APIs (small buffers) - includes the hex dump
 * ------------------------------------------------------------------------ */
static void test_buffer_apis(void){
  uint8_t  wb[32],  rb[32];
  uint16_t wh[8],   rh[8];
  uint32_t ww[8],   rw[8];
  uint32_t ok;

  section("3. Flash_Write*_Buf / Flash_Read*_Buf");
  fresh_page();

  /* ---- byte buf: even addr, even length ---- */
  for (uint32_t i = 0; i < 8; i++) wb[i] = 0xA0 + i;
  ok = Flash_WriteByte_Buf(TEST_BASE + 0x00, wb, 8);
  memset(rb, 0, sizeof(rb));
  Flash_ReadByte_Buf(TEST_BASE + 0x00, rb, 8);
  check("byte buf: even addr, even len (8)", ok && memcmp(wb, rb, 8) == 0);

  /* ---- byte buf: even addr, odd length ---- */
  for (uint32_t i = 0; i < 7; i++) wb[i] = 0xB0 + i;
  ok = Flash_WriteByte_Buf(TEST_BASE + 0x20, wb, 7);
  memset(rb, 0, sizeof(rb));
  Flash_ReadByte_Buf(TEST_BASE + 0x20, rb, 7);
  check("byte buf: even addr, odd len (7)", ok && memcmp(wb, rb, 7) == 0);
  check("byte buf: byte after odd tail is still 0xFF",
        Flash_ReadByte(TEST_BASE + 0x27) == 0xFF);

  /* ---- byte buf: odd addr, even length ---- */
  for (uint32_t i = 0; i < 6; i++) wb[i] = 0xC0 + i;
  ok = Flash_WriteByte_Buf(TEST_BASE + 0x41, wb, 6);
  memset(rb, 0, sizeof(rb));
  Flash_ReadByte_Buf(TEST_BASE + 0x41, rb, 6);
  check("byte buf: odd addr, even len (6)", ok && memcmp(wb, rb, 6) == 0);
  check("byte buf: byte before odd head is still 0xFF",
        Flash_ReadByte(TEST_BASE + 0x40) == 0xFF);
  check("byte buf: byte after odd-addr write is still 0xFF",
        Flash_ReadByte(TEST_BASE + 0x47) == 0xFF);

  /* ---- byte buf: zero length ---- */
  ok = Flash_WriteByte_Buf(TEST_BASE + 0x100, wb, 0);
  check("byte buf: n=0 returns OK and writes nothing",
        ok && Flash_ReadByte(TEST_BASE + 0x100) == 0xFF);

  /* ---- halfword buf ---- */
  for (uint32_t i = 0; i < 8; i++) wh[i] = (uint16_t)(0x1000 + i * 0x111);
  ok = Flash_WriteHlfWord_Buf(TEST_BASE + 0x80, wh, 8);
  memset(rh, 0, sizeof(rh));
  Flash_ReadHlfWord_Buf(TEST_BASE + 0x80, rh, 8);
  check("halfword buf: write 8 + read back", ok && memcmp(wh, rh, sizeof(wh)) == 0);
  check("halfword buf: cross-check with Flash_ReadWord",
        Flash_ReadWord(TEST_BASE + 0x80) == ((uint32_t)wh[1] << 16 | wh[0]));

  /* ---- word buf ---- */
  for (uint32_t i = 0; i < 8; i++) ww[i] = 0xDEADBE00UL + i;
  ok = Flash_WriteWord_Buf(TEST_BASE + 0xC0, ww, 8);
  memset(rw, 0, sizeof(rw));
  Flash_ReadWord_Buf(TEST_BASE + 0xC0, rw, 8);
  check("word buf: write 8 + read back", ok && memcmp(ww, rw, sizeof(ww)) == 0);
  check("word buf: cross-check with Flash_ReadHlfWord",
        Flash_ReadHlfWord(TEST_BASE + 0xC0) == (uint16_t)(ww[0] & 0xFFFF) &&
        Flash_ReadHlfWord(TEST_BASE + 0xC2) == (uint16_t)(ww[0] >> 16));

  /* ---- negative tests: buffer writes over non-blank flash must fail ---- */
  {
    uint16_t bad = 0x1234;   /* TEST_BASE+0x80 already holds 0x1000 */
    ok = Flash_WriteHlfWord_Buf(TEST_BASE + 0x80, &bad, 1);
    check("halfword buf: write over non-erased fails (PGERR)", ok == 0);
    check("halfword buf: failed write left old data intact",
          Flash_ReadHlfWord(TEST_BASE + 0x80) == wh[0]);
  }
  {
    uint32_t badw = 0x12345678UL;   /* TEST_BASE+0xC0 already holds 0xDEADBE00 */
    ok = Flash_WriteWord_Buf(TEST_BASE + 0xC0, &badw, 1);
    check("word buf: write over non-erased fails (PGERR)", ok == 0);
    check("word buf: failed write left old data intact",
          Flash_ReadWord(TEST_BASE + 0xC0) == ww[0]);
  }
  {
    uint8_t badb[2] = { 0x99, 0x98 };   /* TEST_BASE+0x00 already holds A0 A1 */
    ok = Flash_WriteByte_Buf(TEST_BASE + 0x00, badb, 2);
    check("byte buf: write over non-erased fails (PGERR)", ok == 0);
    check("byte buf: failed write left old data intact",
          Flash_ReadByte(TEST_BASE + 0x00) == 0xA0 &&
          Flash_ReadByte(TEST_BASE + 0x01) == 0xA1);
  }
  check("error flags cleared after failed buffer writes", flash_error_flags_clear());
  check("flash is re-locked after buffer writes", flash_locked());

  /* ---- a failure in the middle of a buffer stops the write there ---- */
  {
    /* TEST_BASE+0x84 is already programmed (0x1222). Writing 4 halfwords from
     * +0x7E: +0x7E is blank (ok), +0x80 is programmed (fails) -> stop. */
    uint16_t mix[4] = { 0x7E7E, 0x1111, 0x2222, 0x3333 };
    ok = Flash_WriteHlfWord_Buf(TEST_BASE + 0x7E, mix, 4);
    check("halfword buf: failure mid-buffer returns 0", ok == 0);
    check("  ...halfwords before the failure were written",
          Flash_ReadHlfWord(TEST_BASE + 0x7E) == 0x7E7E);
    check("  ...halfwords after the failure were not",
          Flash_ReadHlfWord(TEST_BASE + 0x80) == wh[0] &&
          Flash_ReadHlfWord(TEST_BASE + 0x82) == wh[1]);
  }

  /* ---- recovery ---- */
  {
    uint16_t good = 0xABCD;
    ok = Flash_WriteHlfWord_Buf(TEST_BASE + 0x140, &good, 1);
    check("recovery: valid write after failed writes",
          ok && Flash_ReadHlfWord(TEST_BASE + 0x140) == 0xABCD);
  }

  /* ---- hex dump of the written area ---- */
  uart_send("\r\nDump %08lX..%08lX:\r\n",
            (unsigned long)TEST_BASE, (unsigned long)(TEST_BASE + 0x150));
  hex_dump(TEST_BASE, 0x150);
}

/* ---------------------------------------------------------------------------
 * 4. Full-page (1 KB) buffer writes
 * ------------------------------------------------------------------------ */
static void test_full_page(void){
  uint32_t ok;

  section("4. Full 1 KB page through each buffer write API");

  /* bytes */
  for (uint32_t i = 0; i < 1024; i++) big.b[i] = (uint8_t)(i * 7 + 3);
  fresh_page();
  ok = Flash_WriteByte_Buf(TEST_BASE, big.b, 1024);
  check("WriteByte_Buf: 1024 bytes returns OK", ok);
  check("WriteByte_Buf: 1024 bytes verify", verify_region(TEST_BASE, big.b, 1024));

  /* halfwords */
  for (uint32_t i = 0; i < 512; i++) big.h[i] = (uint16_t)(0xA500 + i);
  fresh_page();
  ok = Flash_WriteHlfWord_Buf(TEST_BASE, big.h, 512);
  check("WriteHlfWord_Buf: 512 halfwords returns OK", ok);
  check("WriteHlfWord_Buf: 512 halfwords verify", verify_region(TEST_BASE, big.b, 1024));

  /* words */
  for (uint32_t i = 0; i < 256; i++) big.w[i] = 0x5A5A0000UL + i;
  fresh_page();
  ok = Flash_WriteWord_Buf(TEST_BASE, big.w, 256);
  check("WriteWord_Buf: 256 words returns OK", ok);
  check("WriteWord_Buf: 256 words verify", verify_region(TEST_BASE, big.b, 1024));

  /* read the same page back with the word / halfword buffer readers */
  {
    uint32_t rw[16];
    uint16_t rh[16];
    Flash_ReadWord_Buf(TEST_BASE + 0x100, rw, 16);
    Flash_ReadHlfWord_Buf(TEST_BASE + 0x100, rh, 16);
    check("ReadWord_Buf mid-page matches",
          memcmp(rw, &big.w[64], sizeof(rw)) == 0);
    check("ReadHlfWord_Buf mid-page matches",
          memcmp(rh, &big.h[128], sizeof(rh)) == 0);
  }

  check("flash is re-locked after full-page writes", flash_locked());
}

/* ---------------------------------------------------------------------------
 * 5. Erase behaviour
 * ------------------------------------------------------------------------ */
static void test_erase_behaviour(void){
  section("5. Erase behaviour");

  /* the previous section left a full page of data in TEST_BASE */
  check("page is not blank before erase", !page_is_erased(TEST_BASE));

  /* any address inside the page erases the whole page */
  check("ErasePage with a mid-page address returns OK",
        Flash_ErasePage(TEST_BASE + 0x3FF));
  check("  ...the whole page is blank", page_is_erased(TEST_BASE));

  /* an erase touches exactly one page */
  if (page_is_erased(TEST_PAGE_B)) {
    uint32_t ok;
    fresh_page();

    ok = Flash_WriteWord(TEST_PAGE_B + 0x000, 0x11223344UL)
      && Flash_WriteWord(TEST_PAGE_B + 0x3FC, 0x55667788UL)
      && Flash_WriteWord(TEST_BASE   + 0x000, 0xCAFEBABEUL);
    check("wrote a marker word in both pages", ok);

    check("erasing page 64 returns OK", Flash_ErasePage(TEST_BASE));
    check("  ...page 64 is blank", page_is_erased(TEST_BASE));
    check("  ...page 63 first/last words untouched",
          Flash_ReadWord(TEST_PAGE_B + 0x000) == 0x11223344UL &&
          Flash_ReadWord(TEST_PAGE_B + 0x3FC) == 0x55667788UL);

    check("erasing page 63 via mid address returns OK",
          Flash_ErasePage(TEST_PAGE_B + 0x200));
    check("  ...page 63 is blank", page_is_erased(TEST_PAGE_B));
  } else {
    tests_skipped++;
    uart_send("[SKIP] two-page erase test: page 63 (%08lX) is not blank, "
              "firmware probably lives there\r\n", (unsigned long)TEST_PAGE_B);
  }

  check("flash is re-locked after erases", flash_locked());
}

/* ---------------------------------------------------------------------------
 * 6. Known hardware limitations, demonstrated
 * ------------------------------------------------------------------------ */
static void test_limitations(void){
  uint8_t one = 0x11;
  uint32_t ok;

  section("6. Hardware limitations (expected behaviour)");
  fresh_page();

  /* an odd single byte is stored as 0x11FF in the halfword at +0 */
  ok = Flash_WriteByte_Buf(TEST_BASE + 0x01, &one, 1);
  check("write one byte at odd address", ok && Flash_ReadHlfWord(TEST_BASE) == 0x11FF);

  /* its neighbour lives in the same halfword -> cannot be written separately */
  ok = Flash_WriteByte(TEST_BASE + 0x00, 0x22);
  check("neighbour byte in same halfword cannot be written (expected fail)", ok == 0);
  check("  ...halfword unchanged", Flash_ReadHlfWord(TEST_BASE) == 0x11FF);

  /* after an erase the pair can be written together as one halfword */
  fresh_page();
  {
    uint8_t pair[2] = { 0x22, 0x11 };
    ok = Flash_WriteByte_Buf(TEST_BASE, pair, 2);
    check("both bytes written in one call works",
          ok && Flash_ReadHlfWord(TEST_BASE) == 0x1122);
  }

  /* informational: the F1 flash allows programming 0x0000 over programmed data */
  ok = Flash_WriteHlfWord(TEST_BASE, 0x0000);
  uart_send("[INFO] programming 0x0000 over a programmed halfword: %s, reads %04X\r\n",
            ok ? "accepted" : "rejected", (unsigned)Flash_ReadHlfWord(TEST_BASE));
}

static void run_flash_tests(void){
  uart_send("\r\n=== STM32F103 flash library demonstration ===\r\n");
  uart_send("Scratch page: %08lX..%08lX\r\n",
            (unsigned long)FLASH_64TH_PAGE_START_ADDR,
            (unsigned long)FLASH_64TH_PAGE_END_ADDR);

  test_erase_and_read();
  test_single_writes();
  test_buffer_apis();
  test_full_page();
  test_erase_behaviour();
  test_limitations();

  section("7. Cleanup");
  check("final erase", Flash_ErasePage(TEST_BASE));
  check("page reads blank after final erase", page_is_erased(TEST_BASE));
  check("flash is locked at the end", flash_locked());

  uart_send("\r\n=== Done: %lu passed, %lu failed, %lu skipped ===\r\n",
            (unsigned long)tests_passed,
            (unsigned long)tests_failed,
            (unsigned long)tests_skipped);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);

  HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t *)uart_rx_buf, sizeof(uart_rx_buf) - 1);

  HAL_Delay(500);   /* let the serial terminal attach */
  run_flash_tests();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /* LED: fast blink = all tests passed, slow blink = at least one failed */
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  HAL_Delay(tests_failed ? 1000 : 150);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        uart_rx_buf[Size] = '\0';
        memcpy(coms_processing_buf, uart_rx_buf, Size + 1);
        HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t *)uart_rx_buf, sizeof(uart_rx_buf) - 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {  // fixed the uart bug
  if (huart->Instance == USART1) {
	  HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t *)uart_rx_buf, sizeof(uart_rx_buf) - 1);
  }
}
/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
