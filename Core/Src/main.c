/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : DS2485 -> DS28E18 RUN_SEQ sweep with POR clear
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

#include <string.h>
#include <stdio.h>

/* Private define ------------------------------------------------------------*/
#define DS2485_ADDR_7BIT              0x40U
#define DS2485_ADDR_8BIT              (DS2485_ADDR_7BIT << 1)

#define DS2485_CMD_READ_CFG           0x52U
#define DS2485_CMD_WRITE_CFG          0x99U
#define DS2485_CMD_MASTER_RESET       0x62U
#define DS2485_CMD_1WIRE_BLOCK        0xABU
#define DS2485_CMD_FULL_OW_SEQ        0x67U

#define DS2485_REG_MASTER_CFG         0x00U
#define DS2485_REG_tRSTL              0x01U
#define DS2485_REG_tMSI               0x02U
#define DS2485_REG_tMSP               0x03U
#define DS2485_REG_tRSTH              0x04U
#define DS2485_REG_RPUP_BUF           0x11U
#define DS2485_REG_PDSLEW             0x12U

#define OW_BLOCK_FLAG_RESET           0x01U
#define OW_ROM_CMD_READ_ROM           0x33U

#define DS28E18_CMD_WRITE_SEQ         0x11U
#define DS28E18_CMD_READ_SEQ          0x22U
#define DS28E18_CMD_RUN_SEQ           0x33U
#define DS28E18_CMD_DEVICE_STATUS     0x7AU
#define DS28E18_CMD_WRITE_GPIO_CFG    0x83U


#define DS28E18_CFG_TARGET_GPIO_CTRL  0x0BU
#define DS28E18_CFG_MODULE_GPIO       0x03U
#define DS28E18_GPIO_CTRL_HI          0xA5U
#define DS28E18_GPIO_CTRL_LO          0x0FU

#define DS28E18_SEQ_ADDR0             0x000U

#define DS28E18_SEQ_I2C_START         0x02U
#define DS28E18_SEQ_I2C_STOP          0x03U
#define DS28E18_SEQ_I2C_WRITE         0xE3U
#define DS28E18_SEQ_I2C_READ          0xD4U
#define DS28E18_SEQ_DELAY             0xDDU
#define DS28E18_SEQ_SENS_VDD_ON       0xCCU
#define DS28E18_SEQ_SENS_VDD_OFF      0xBBU

#define LSM6DSO_ADDR0_7BIT            0x6AU
#define LSM6DSO_REG_WHO_AM_I          0x0FU

#define APP_BUF_MAX                   192U
#define FIXED_FULLSEQ_DELAY_STEPS     0x20U
#define POWER_ON_DELAY_CODE           0x05U

/* Private variables ---------------------------------------------------------*/
static uint8_t g_fullseq_delay_steps = FIXED_FULLSEQ_DELAY_STEPS;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void uart_write(const char *s);
static void uart_write_line(const char *s);
static void uart_write_hex8(uint8_t value);
static void uart_write_hex16(uint16_t value);
static void uart_dump_bytes(const char *label, const uint8_t *buf, uint16_t len);

static HAL_StatusTypeDef ds2485_is_ready(void);
static HAL_StatusTypeDef ds2485_tx_then_rx_poll(const uint8_t *tx, uint16_t tx_len,
                                                uint8_t *rx, uint16_t rx_len,
                                                uint32_t rx_timeout_ms);
static HAL_StatusTypeDef ds2485_master_reset(uint8_t *result);
static HAL_StatusTypeDef ds2485_read_cfg_u16(uint8_t reg, uint16_t *value_out, uint8_t *result);
static HAL_StatusTypeDef ds2485_write_cfg_u16(uint8_t reg, uint16_t value, uint8_t *result);
static HAL_StatusTypeDef ds2485_1wire_block(uint8_t flags,
                                            const uint8_t *ow_tx, uint8_t ow_tx_len,
                                            uint8_t *rx, uint16_t rx_len,
                                            uint32_t rx_timeout_ms);
static HAL_StatusTypeDef ds2485_full_ow_sequence(uint8_t delay_steps,
                                                 const uint8_t rom_id[8],
                                                 const uint8_t *ow_data,
                                                 uint8_t ow_data_len,
                                                 uint8_t *rx,
                                                 uint16_t rx_len,
                                                 uint32_t rx_timeout_ms);

static uint8_t crc8_maxim(const uint8_t *data, uint8_t len);
static uint8_t is_placeholder_rom(const uint8_t rom[8]);

static uint8_t app_init_and_config(void);
static uint8_t app_read_rom_once(uint8_t rom_out[8], uint8_t print_raw);
static uint8_t app_read_rom_retry(uint8_t rom_out[8], uint8_t tries, const char *tag_prefix);
static uint8_t app_ds28e18_write_gpio_cfg(const uint8_t rom[8]);
static uint8_t app_ds28e18_device_status(const uint8_t rom[8],
                                         uint8_t *dev_status,
                                         uint8_t *version,
                                         uint8_t *manid_lo,
                                         uint8_t *manid_hi);

static uint8_t app_ds28e18_write_seq(const uint8_t rom[8],
                                     uint16_t addr,
                                     const uint8_t *data,
                                     uint8_t data_len);
static uint8_t app_ds28e18_write_seq_retry(const uint8_t rom[8],
                                           uint16_t addr,
                                           const uint8_t *data,
                                           uint8_t data_len,
                                           uint8_t tries);
static uint8_t app_ds28e18_read_seq(const uint8_t rom[8],
                                    uint16_t addr,
                                    uint8_t slen,
                                    uint8_t *data_out,
                                    uint8_t *out_count);
static uint8_t app_ds28e18_run_seq(const uint8_t rom[8],
                                   uint16_t addr,
                                   uint16_t slen,
                                   uint8_t *result_byte,
                                   uint16_t *nack_offset);

static void test_one_sequence(const uint8_t rom[8],
                              const char *name,
                              const uint8_t *seq,
                              uint8_t seq_len);

static uint8_t ds28e18_poweron_via_skip_rom(void);

/* UART ----------------------------------------------------------------------*/
static void uart_write(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

static void uart_write_line(const char *s)
{
  uart_write(s);
  uart_write("\r\n");
}

static void uart_write_hex8(uint8_t value)
{
  char msg[5];
  snprintf(msg, sizeof(msg), "%02X", value);
  uart_write(msg);
}

static void uart_write_hex16(uint16_t value)
{
  char msg[7];
  snprintf(msg, sizeof(msg), "%04X", value);
  uart_write(msg);
}

static void uart_dump_bytes(const char *label, const uint8_t *buf, uint16_t len)
{
  uint16_t i;

  uart_write(label);
  for (i = 0U; i < len; i++)
  {
    if (i != 0U) uart_write(" ");
    uart_write_hex8(buf[i]);
  }
  uart_write("\r\n");
}

/* DS2485 low-level ----------------------------------------------------------*/
static HAL_StatusTypeDef ds2485_is_ready(void)
{
  return HAL_I2C_IsDeviceReady(&hi2c1, DS2485_ADDR_8BIT, 5, 100);
}

static HAL_StatusTypeDef ds2485_tx_then_rx_poll(const uint8_t *tx, uint16_t tx_len,
                                                uint8_t *rx, uint16_t rx_len,
                                                uint32_t rx_timeout_ms)
{
  HAL_StatusTypeDef ret;
  uint32_t t0;

  ret = HAL_I2C_Master_Transmit(&hi2c1, DS2485_ADDR_8BIT, (uint8_t *)tx, tx_len, 200);
  if (ret != HAL_OK) return ret;

  t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < rx_timeout_ms)
  {
    ret = HAL_I2C_Master_Receive(&hi2c1, DS2485_ADDR_8BIT, rx, rx_len, 200);
    if (ret == HAL_OK) return HAL_OK;
    HAL_Delay(1);
  }

  return ret;
}

static HAL_StatusTypeDef ds2485_master_reset(uint8_t *result)
{
  uint8_t tx = DS2485_CMD_MASTER_RESET;
  uint8_t rx[2];

  if (ds2485_tx_then_rx_poll(&tx, 1U, rx, sizeof(rx), 20) != HAL_OK) return HAL_ERROR;
  if (result) *result = rx[1];
  return HAL_OK;
}

static HAL_StatusTypeDef ds2485_read_cfg_u16(uint8_t reg, uint16_t *value_out, uint8_t *result)
{
  uint8_t tx[3];
  uint8_t rx[4];

  tx[0] = DS2485_CMD_READ_CFG;
  tx[1] = 0x01U;
  tx[2] = reg;

  if (ds2485_tx_then_rx_poll(tx, sizeof(tx), rx, sizeof(rx), 20) != HAL_OK) return HAL_ERROR;

  if (result) *result = rx[1];
  if (value_out) *value_out = (uint16_t)rx[2] | ((uint16_t)rx[3] << 8);

  return HAL_OK;
}

static HAL_StatusTypeDef ds2485_write_cfg_u16(uint8_t reg, uint16_t value, uint8_t *result)
{
  uint8_t tx[5];
  uint8_t rx[2];

  tx[0] = DS2485_CMD_WRITE_CFG;
  tx[1] = 0x03U;
  tx[2] = reg;
  tx[3] = (uint8_t)(value & 0xFFU);
  tx[4] = (uint8_t)((value >> 8) & 0xFFU);

  if (ds2485_tx_then_rx_poll(tx, sizeof(tx), rx, sizeof(rx), 20) != HAL_OK) return HAL_ERROR;

  if (result) *result = rx[1];
  return HAL_OK;
}

static HAL_StatusTypeDef ds2485_1wire_block(uint8_t flags,
                                            const uint8_t *ow_tx, uint8_t ow_tx_len,
                                            uint8_t *rx, uint16_t rx_len,
                                            uint32_t rx_timeout_ms)
{
  uint8_t tx[3 + APP_BUF_MAX];

  if ((ow_tx_len == 0U) || (ow_tx_len > APP_BUF_MAX)) return HAL_ERROR;

  tx[0] = DS2485_CMD_1WIRE_BLOCK;
  tx[1] = (uint8_t)(1U + ow_tx_len);
  tx[2] = flags;
  memcpy(&tx[3], ow_tx, ow_tx_len);

  return ds2485_tx_then_rx_poll(tx, (uint16_t)(3U + ow_tx_len), rx, rx_len, rx_timeout_ms);
}

static HAL_StatusTypeDef ds2485_full_ow_sequence(uint8_t delay_steps,
                                                 const uint8_t rom_id[8],
                                                 const uint8_t *ow_data,
                                                 uint8_t ow_data_len,
                                                 uint8_t *rx,
                                                 uint16_t rx_len,
                                                 uint32_t rx_timeout_ms)
{
  uint8_t tx[3 + 1 + 8 + APP_BUF_MAX];

  if ((ow_data_len == 0U) || (ow_data_len > 116U)) return HAL_ERROR;

  tx[0] = 0x67U;
  tx[1] = (uint8_t)(1U + 8U + ow_data_len);
  tx[2] = delay_steps;
  memcpy(&tx[3], rom_id, 8U);
  memcpy(&tx[11], ow_data, ow_data_len);

  return ds2485_tx_then_rx_poll(tx, (uint16_t)(11U + ow_data_len), rx, rx_len, rx_timeout_ms);
}

/* Utility -------------------------------------------------------------------*/
static uint8_t crc8_maxim(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0U, i, j;

  for (i = 0U; i < len; i++)
  {
    crc ^= data[i];
    for (j = 0U; j < 8U; j++)
    {
      if (crc & 0x01U) crc = (uint8_t)((crc >> 1) ^ 0x8CU);
      else             crc >>= 1;
    }
  }
  return crc;
}

static uint8_t is_placeholder_rom(const uint8_t rom[8])
{
  uint8_t i;
  uint8_t all_zero = 1U;
  uint8_t all_ff = 1U;

  if (rom == NULL) return 1U;

  for (i = 0U; i < 8U; i++)
  {
    if (rom[i] != 0x00U) all_zero = 0U;
    if (rom[i] != 0xFFU) all_ff = 0U;
  }

  if (all_zero || all_ff) return 1U;
  if (rom[0] == 0x00U) return 1U;

  return 0U;
}

/* App -----------------------------------------------------------------------*/
static uint8_t app_init_and_config(void)
{
  uint8_t result = 0xFFU;
  uint16_t rd = 0x0000U;

  uart_write("[1] Probing DS2485 ... ");
  if (ds2485_is_ready() != HAL_OK)
  {
    uart_write_line("FAIL");
    return 0U;
  }
  uart_write_line("ACK OK");

  uart_write("[2] Master reset ... ");
  if (ds2485_master_reset(&result) != HAL_OK)
  {
    uart_write_line("I2C FAIL");
    return 0U;
  }
  uart_write("result=0x"); uart_write_hex8(result); uart_write("\r\n");
  if (result != 0xAAU) return 0U;

  HAL_Delay(10);

  if (ds2485_write_cfg_u16(DS2485_REG_RPUP_BUF, 0x8026U, &result) != HAL_OK) return 0U;
  if (result != 0xAAU) return 0U;

  if (ds2485_write_cfg_u16(DS2485_REG_MASTER_CFG, 0x3000U, &result) != HAL_OK) return 0U;
  if (result != 0xAAU) return 0U;

  if (ds2485_write_cfg_u16(DS2485_REG_PDSLEW, 0x0006U, &result) != HAL_OK) return 0U;
  if (result != 0xAAU) return 0U;

  if (ds2485_write_cfg_u16(DS2485_REG_tRSTL, 0xA080U, &result) != HAL_OK) return 0U;
  if (result != 0xAAU) return 0U;

  if (ds2485_write_cfg_u16(DS2485_REG_tMSI, 0x8060U, &result) != HAL_OK) return 0U;
  if (result != 0xAAU) return 0U;

  if (ds2485_write_cfg_u16(DS2485_REG_tMSP, 0x8460U, &result) != HAL_OK) return 0U;
  if (result != 0xAAU) return 0U;

  if (ds2485_write_cfg_u16(DS2485_REG_tRSTH, 0xA080U, &result) != HAL_OK) return 0U;
  if (result != 0xAAU) return 0U;

  uart_write_line("[3] DS2485 config write done");

  if (ds2485_read_cfg_u16(DS2485_REG_RPUP_BUF, &rd, &result) == HAL_OK)
  {
    uart_write("[CFG ] RPUP/BUF = 0x"); uart_write_hex16(rd);
    uart_write(" result=0x"); uart_write_hex8(result);
    uart_write("\r\n");
  }

  if (ds2485_read_cfg_u16(DS2485_REG_MASTER_CFG, &rd, &result) == HAL_OK)
  {
    uart_write("[CFG ] MASTER   = 0x"); uart_write_hex16(rd);
    uart_write(" result=0x"); uart_write_hex8(result);
    uart_write("\r\n");
  }

  uart_write("[CFG ] fullseq delay steps = 0x");
  uart_write_hex8(g_fullseq_delay_steps);
  uart_write("\r\n");

  return 1U;
}

static uint8_t app_read_rom_once(uint8_t rom_out[8], uint8_t print_raw)
{
  uint8_t ow_tx[9];
  uint8_t rx[11];
  uint8_t i;

  ow_tx[0] = OW_ROM_CMD_READ_ROM;
  for (i = 1U; i < sizeof(ow_tx); i++) ow_tx[i] = 0xFFU;

  memset(rx, 0xFF, sizeof(rx));
  if (ds2485_1wire_block(OW_BLOCK_FLAG_RESET, ow_tx, (uint8_t)sizeof(ow_tx), rx, sizeof(rx), 100) != HAL_OK)
  {
    uart_write_line("[ROM ] RX FAIL");
    return 0U;
  }

  uart_write("[ROM ] len=0x"); uart_write_hex8(rx[0]);
  uart_write(" result=0x");    uart_write_hex8(rx[1]);
  uart_write("\r\n");

  if (print_raw) uart_dump_bytes("[ROM ] raw   = ", rx, sizeof(rx));

  if (rx[1] != 0xAAU) return 0U;

  memcpy(rom_out, &rx[3], 8U);
  uart_dump_bytes("[ROM ] id    = ", rom_out, 8U);

  if (crc8_maxim(rom_out, 7U) != rom_out[7])
  {
    uart_write_line("[ROM ] CRC FAIL");
    return 0U;
  }

  uart_write_line("[ROM ] CRC OK");
  return 1U;
}

static uint8_t app_read_rom_retry(uint8_t rom_out[8], uint8_t tries, const char *tag_prefix)
{
  uint8_t i;

  for (i = 0U; i < tries; i++)
  {
    uart_write(tag_prefix);
    uart_write(" try ");
    uart_write_hex8((uint8_t)(i + 1U));
    uart_write("\r\n");

    if (app_read_rom_once(rom_out, 1U))
      return 1U;

    HAL_Delay(40);
  }

  return 0U;
}

/* DS28E18 power-up init using Skip ROM (0xCC).
   Must be called before the real ROM ID is known.
   Per datasheet p.13: issue twice. Ignore result on first call. */
static uint8_t ds28e18_poweron_via_skip_rom(void)
{
  /* 1-Wire payload:
     [0]    Skip ROM (0xCC)
     [1]    Command Start (0x66)
     [2]    Length = 5 (Write GPIO Config cmd + 4 params)
     [3]    DS28E18_CMD_WRITE_GPIO_CFG (0x83)
     [4]    DS28E18_CFG_TARGET_GPIO_CTRL (0x0B)
     [5]    DS28E18_CFG_MODULE_GPIO (0x03)
     [6]    DS28E18_GPIO_CTRL_HI (0xA5)
     [7]    DS28E18_GPIO_CTRL_LO (0x0F)
     [8..9] 0xFF 0xFF  <- CRC16 rx slots (DS28E18 sends these back)
     [10]   0xAAU      <- Release byte (tells DS28E18 to execute)
     [11]   0xFF       <- Dummy byte rx slot
     [12]   0xFF       <- Result length rx slot
     [13]   0xFF       <- Result byte rx slot
     [14..15] 0xFF 0xFF <- Result CRC16 rx slots                   */

  uint8_t ow_tx[16] = {
    0xCCU,                          /* Skip ROM              */
    0x66U,                          /* Command Start         */
    0x05U,                          /* Length = 5            */
    DS28E18_CMD_WRITE_GPIO_CFG,     /* 0x83                  */
    DS28E18_CFG_TARGET_GPIO_CTRL,   /* 0x0B                  */
    DS28E18_CFG_MODULE_GPIO,        /* 0x03                  */
    DS28E18_GPIO_CTRL_HI,           /* 0xA5                  */
    DS28E18_GPIO_CTRL_LO,           /* 0x0F                  */
    0xFFU, 0xFFU,                   /* CRC16 rx slots        */
    0xAAU,                          /* Release byte          */
    0xFFU,                          /* Dummy byte rx slot    */
    0xFFU,                          /* Result length rx slot */
    0xFFU,                          /* Result byte rx slot   */
    0xFFU, 0xFFU                    /* Result CRC16 rx slots */
  };
  uint8_t rx[18];   /* 2 (ds2485 hdr) + 16 (echoed ow_tx) */

  memset(rx, 0xFF, sizeof(rx));

  if (ds2485_1wire_block(OW_BLOCK_FLAG_RESET,
                         ow_tx, (uint8_t)sizeof(ow_tx),
                         rx, (uint16_t)sizeof(rx), 400U) != HAL_OK)
  {
    uart_write_line("[SKIP] 1-Wire block FAIL");
    return 0U;
  }

  /* rx[0]  = DS2485 length byte
     rx[1]  = DS2485 result (0xAA = OK)
     rx[2..17] = echoed ow_tx bytes
     Result byte from DS28E18 is at rx[2 + 13] = rx[15] */
  uart_write("[SKIP] ds2485=0x"); uart_write_hex8(rx[1]);
  uart_write(" ds28e18_res=0x"); uart_write_hex8(rx[15]);
  uart_write("\r\n");

  return (rx[1] == 0xAAU) ? 1U : 0U;
}

static uint8_t app_ds28e18_write_gpio_cfg(const uint8_t rom[8])
{
  uint8_t ow_data[5] = {
    DS28E18_CMD_WRITE_GPIO_CFG,
    DS28E18_CFG_TARGET_GPIO_CTRL,
    DS28E18_CFG_MODULE_GPIO,
    DS28E18_GPIO_CTRL_HI,
    DS28E18_GPIO_CTRL_LO
  };
  uint8_t rx[4];

  memset(rx, 0xFF, sizeof(rx));
  if (ds2485_full_ow_sequence(g_fullseq_delay_steps, rom, ow_data, sizeof(ow_data), rx, sizeof(rx), 300) != HAL_OK)
  {
    uart_write_line("[GPIO-W] RX FAIL");
    return 0U;
  }

  uart_write("[GPIO-W] len=0x"); uart_write_hex8(rx[0]);
  uart_write(" ds2485=0x");      uart_write_hex8(rx[1]);
  uart_write(" ow_len=0x");      uart_write_hex8(rx[2]);
  uart_write(" ow_res=0x");      uart_write_hex8(rx[3]);
  uart_write("\r\n");

  return ((rx[1] == 0xAAU) && (rx[2] == 0x01U) && (rx[3] == 0xAAU)) ? 1U : 0U;
}

static uint8_t app_ds28e18_device_status(const uint8_t rom[8],
                                         uint8_t *dev_status,
                                         uint8_t *version,
                                         uint8_t *manid_lo,
                                         uint8_t *manid_hi)
{
  uint8_t ow_data[1];
  uint8_t rx[8];

  ow_data[0] = DS28E18_CMD_DEVICE_STATUS;

  memset(rx, 0xFF, sizeof(rx));
  if (ds2485_full_ow_sequence(g_fullseq_delay_steps, rom,
                              ow_data, sizeof(ow_data),
                              rx, sizeof(rx), 300) != HAL_OK)
  {
    uart_write_line("[STAT ] RX FAIL");
    return 0U;
  }

  uart_write("[STAT ] len=0x"); uart_write_hex8(rx[0]);
  uart_write(" ds2485=0x");     uart_write_hex8(rx[1]);
  uart_write(" ow_len=0x");     uart_write_hex8(rx[2]);
  uart_write(" ow_res=0x");     uart_write_hex8(rx[3]);
  if (rx[2] >= 5U)
  {
    uart_write(" st=0x");       uart_write_hex8(rx[4]);
    uart_write(" ver=0x");      uart_write_hex8(rx[5]);
    uart_write(" manid_lo=0x"); uart_write_hex8(rx[6]);
    uart_write(" manid_hi=0x"); uart_write_hex8(rx[7]);
  }
  uart_write("\r\n");

  if ((rx[1] != 0xAAU) || (rx[2] != 5U) || (rx[3] != 0xAAU))
    return 0U;

  if (dev_status) *dev_status = rx[4];
  if (version)    *version    = rx[5];
  if (manid_lo)   *manid_lo   = rx[6];
  if (manid_hi)   *manid_hi   = rx[7];

  return 1U;
}

/* DS28E18 seq wrappers ------------------------------------------------------*/
static uint8_t app_ds28e18_write_seq(const uint8_t rom[8],
                                     uint16_t addr,
                                     const uint8_t *data,
                                     uint8_t data_len)
{
  uint8_t ow_data[3 + 128];
  uint8_t rx[4];

  if (data_len > 128U) return 0U;

  ow_data[0] = DS28E18_CMD_WRITE_SEQ;
  ow_data[1] = (uint8_t)(addr & 0xFFU);
  ow_data[2] = (uint8_t)((addr >> 8) & 0x01U);
  memcpy(&ow_data[3], data, data_len);

  memset(rx, 0xFF, sizeof(rx));
  if (ds2485_full_ow_sequence(g_fullseq_delay_steps, rom, ow_data,
                              (uint8_t)(3U + data_len), rx, sizeof(rx), 350) != HAL_OK)
  {
    uart_write_line("[SEQ-W] RX FAIL");
    return 0U;
  }

  uart_write("[SEQ-W] len=0x"); uart_write_hex8(rx[0]);
  uart_write(" ds2485=0x");     uart_write_hex8(rx[1]);
  uart_write(" ow_len=0x");     uart_write_hex8(rx[2]);
  uart_write(" ow_res=0x");     uart_write_hex8(rx[3]);
  uart_write("\r\n");

  return ((rx[1] == 0xAAU) && (rx[2] == 0x01U) && (rx[3] == 0xAAU)) ? 1U : 0U;
}

static uint8_t app_ds28e18_write_seq_retry(const uint8_t rom[8],
                                           uint16_t addr,
                                           const uint8_t *data,
                                           uint8_t data_len,
                                           uint8_t tries)
{
  uint8_t i;

  for (i = 0U; i < tries; i++)
  {
    if (app_ds28e18_write_seq(rom, addr, data, data_len))
      return 1U;
    HAL_Delay(20);
  }

  return 0U;
}

static uint8_t app_ds28e18_read_seq(const uint8_t rom[8],
                                    uint16_t addr,
                                    uint8_t slen,
                                    uint8_t *data_out,
                                    uint8_t *out_count)
{
  uint8_t ow_data[3];
  uint8_t rx[132];
  uint16_t rx_len;
  uint8_t payload_count;

  ow_data[0] = DS28E18_CMD_READ_SEQ;
  ow_data[1] = (uint8_t)(addr & 0xFFU);
  ow_data[2] = (uint8_t)(((slen & 0x7FU) << 1) | ((addr >> 8) & 0x01U));

  rx_len = (uint16_t)(4U + slen);   // len + ds2485_result + OW_RSLT_LEN + ds28e18_result + data

  memset(rx, 0xFF, sizeof(rx));
  if (ds2485_full_ow_sequence(g_fullseq_delay_steps, rom, ow_data,
                              sizeof(ow_data), rx, rx_len, 350) != HAL_OK)
  {
    uart_write_line("[SEQ-RD ] RX FAIL");
    return 0U;
  }

  uart_write("[SEQ-RD ] len=0x"); uart_write_hex8(rx[0]);
  uart_write(" ds2485=0x");       uart_write_hex8(rx[1]);
  uart_write(" ow_len=0x");       uart_write_hex8(rx[2]);
  uart_write(" ow_res=0x");       uart_write_hex8(rx[3]);
  uart_write("\r\n");

  payload_count = (rx[2] >= 1U) ? (uint8_t)(rx[2] - 1U) : 0U;

  if ((rx[1] == 0xAAU) && (rx[3] == 0xAAU))
  {
    if ((data_out != NULL) && (payload_count > 0U)) memcpy(data_out, &rx[4], payload_count);
    if (out_count) *out_count = payload_count;
    return 1U;
  }

  return 0U;
}

static uint8_t app_ds28e18_run_seq(const uint8_t rom[8],
                                   uint16_t addr,
                                   uint16_t slen,
                                   uint8_t *result_byte,
                                   uint16_t *nack_offset)
{
  uint8_t ow_data[4];
  uint8_t rx[6];

  ow_data[0] = DS28E18_CMD_RUN_SEQ;
  ow_data[1] = (uint8_t)(addr & 0xFFU);
  ow_data[2] = (uint8_t)(((slen & 0x7FU) << 1) | ((addr >> 8) & 0x01U));
  ow_data[3] = (uint8_t)((slen >> 7) & 0x03U);

  memset(rx, 0xFF, sizeof(rx));
  if (ds2485_full_ow_sequence(g_fullseq_delay_steps, rom, ow_data, sizeof(ow_data),
                              rx, sizeof(rx), 450) != HAL_OK)
  {
    uart_write_line("[SEQ-RUN] RX FAIL");
    return 0U;
  }

  uart_write("[SEQ-RUN] len=0x"); uart_write_hex8(rx[0]);
  uart_write(" ds2485=0x");       uart_write_hex8(rx[1]);
  uart_write(" ow_len=0x");       uart_write_hex8(rx[2]);
  uart_write(" ow_res=0x");       uart_write_hex8(rx[3]);
  if (rx[2] >= 3U)
  {
    uart_write(" snack_lo=0x");   uart_write_hex8(rx[4]);
    uart_write(" snack_hi=0x");   uart_write_hex8(rx[5]);
  }
  uart_write("\r\n");

  if (result_byte) *result_byte = rx[3];

  if ((rx[1] == 0xAAU) && (rx[2] == 3U) && (rx[3] == 0x88U) && nack_offset)
    *nack_offset = ((uint16_t)rx[5] << 8) | rx[4];

  return (rx[1] == 0xAAU) ? 1U : 0U;
}

/* Tests ---------------------------------------------------------------------*/
static void test_one_sequence(const uint8_t rom[8],
                              const char *name,
                              const uint8_t *seq,
                              uint8_t seq_len)
{
  uint8_t dump[48];
  uint8_t dump_len = 0U;
  uint8_t run_res = 0xFFU;
  uint16_t nack = 0xFFFFU;

  uart_write_line("================================");
  uart_write(name);
  uart_write("\r\n");
  uart_write_line("================================");

  uart_dump_bytes("[SEQ ] write = ", seq, seq_len);

  if (!app_ds28e18_write_seq_retry(rom, DS28E18_SEQ_ADDR0, seq, seq_len, 3U))
  {
    uart_write_line("[ERR] WRITE_SEQ fail");
    return;
  }

  if (app_ds28e18_read_seq(rom, DS28E18_SEQ_ADDR0, seq_len, dump, &dump_len))
  {
    uart_dump_bytes("[SEQ ] read  = ", dump, dump_len);
  }

  HAL_Delay(10);

  if (!app_ds28e18_run_seq(rom, DS28E18_SEQ_ADDR0, seq_len, &run_res, &nack))
  {
    uart_write_line("[ERR] RUN transport fail");
    return;
  }

  if (run_res == 0xAAU)
  {
    uart_write_line("[HIT ] sequence executed OK");
  }
  else if (run_res == 0x88U)
  {
    uart_write("[INFO] NACK offset=0x");
    uart_write_hex16(nack);
    uart_write("\r\n");
  }
  else
  {
    uart_write("[INFO] run_res=0x");
    uart_write_hex8(run_res);
    uart_write("\r\n");
  }
}

/* Main ----------------------------------------------------------------------*/
int main(void)
{
  uint8_t rom[8];
  uint8_t ds = 0xFFU, ver = 0xFFU, midl = 0xFFU, midh = 0xFFU;

  uint8_t seq1[] = { DS28E18_SEQ_DELAY, 0x05U };
  uint8_t seq2[] = { DS28E18_SEQ_I2C_START, DS28E18_SEQ_I2C_STOP };
  uint8_t seq3[] = { DS28E18_SEQ_SENS_VDD_ON, DS28E18_SEQ_DELAY, POWER_ON_DELAY_CODE };
  uint8_t seq4[] = { DS28E18_SEQ_SENS_VDD_ON, DS28E18_SEQ_DELAY, POWER_ON_DELAY_CODE,
                     DS28E18_SEQ_I2C_START, DS28E18_SEQ_I2C_STOP };
  uint8_t seq5[] = { DS28E18_SEQ_SENS_VDD_ON, DS28E18_SEQ_DELAY, POWER_ON_DELAY_CODE,
                     DS28E18_SEQ_I2C_START, DS28E18_SEQ_I2C_WRITE, 0x01U,
                     (uint8_t)(LSM6DSO_ADDR0_7BIT << 1), DS28E18_SEQ_I2C_STOP };
  uint8_t seq6[] = { DS28E18_SEQ_SENS_VDD_ON, DS28E18_SEQ_DELAY, POWER_ON_DELAY_CODE,
                     DS28E18_SEQ_I2C_START,
                     DS28E18_SEQ_I2C_WRITE, 0x02U, (uint8_t)(LSM6DSO_ADDR0_7BIT << 1), LSM6DSO_REG_WHO_AM_I,
                     DS28E18_SEQ_I2C_START,
                     DS28E18_SEQ_I2C_WRITE, 0x01U, (uint8_t)((LSM6DSO_ADDR0_7BIT << 1) | 1U),
                     DS28E18_SEQ_I2C_READ, 0x01U,
                     DS28E18_SEQ_I2C_STOP };

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  HAL_Delay(200);

  uart_write_line("================================");
  uart_write_line(" UART OK - DS2485 -> DS28E18 -> RUN_SEQ SWEEP");
  uart_write_line("================================");

  if (!app_init_and_config())
  {
    uart_write_line("[ERR] init/config fail");
    while (1) { HAL_Delay(500); }
  }

  uart_write_line("-------");

  /* --- DS28E18 power-up sequence (datasheet p.13) ---
       On every power-up the ROM reads 56 00 00 00 00 00 00 B2.
       Two Skip ROM + Write GPIO Config calls load the real ROM ID. */

    uart_write_line("[4] DS28E18 power-up init call #1 (result may be invalid)");
    ds28e18_poweron_via_skip_rom();   /* ignore return - first call is always unreliable */
    HAL_Delay(10U);

    uart_write_line("[5] DS28E18 power-up init call #2");
    if (!ds28e18_poweron_via_skip_rom())
    {
      uart_write_line("[ERR] DS28E18 power-up init failed");
      while (1) { HAL_Delay(500); }
    }
    HAL_Delay(10U);

    uart_write_line("[6] READ ROM (expecting real unique ID now)");
    if (!app_read_rom_retry(rom, 5U, "[ROM]"))
    {
      uart_write_line("[ERR] READ ROM failed after power-up init");
      while (1) { HAL_Delay(500); }
    }

    uart_write("[INFO] using ROM = ");
    uart_dump_bytes("", rom, 8U);

    uart_write_line("[7] DEVICE_STATUS (clears POR)");
    if (!app_ds28e18_device_status(rom, &ds, &ver, &midl, &midh))
    {
      uart_write_line("[ERR] DEVICE_STATUS failed");
      while (1) { HAL_Delay(500); }
    }

    uart_write("[INFO] device status byte = 0x");
    uart_write_hex8(ds);
    uart_write("\r\n");
  uart_write("[INFO] device status byte = 0x");
  uart_write_hex8(ds);
  uart_write("\r\n");

  uart_write("[INFO] using ROM = ");
  uart_dump_bytes("", rom, 8U);

  uart_write_line("-------");

  test_one_sequence(rom, "TEST1: DELAY ONLY", seq1, sizeof(seq1));
  test_one_sequence(rom, "TEST2: I2C START + STOP", seq2, sizeof(seq2));
  test_one_sequence(rom, "TEST3: SENS_VDD_ON + DELAY", seq3, sizeof(seq3));
  test_one_sequence(rom, "TEST4: PWR_ON + DELAY + I2C START + STOP", seq4, sizeof(seq4));
  test_one_sequence(rom, "TEST5: PWR_ON + DELAY + I2C START + WRITE(0xD4) + STOP", seq5, sizeof(seq5));
  test_one_sequence(rom, "TEST6: PWR_ON + DELAY + WHO_AM_I READ", seq6, sizeof(seq6));

  uart_write_line("================================");
  uart_write_line(" DONE");
  uart_write_line("================================");

  while (1)
  {
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
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

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK
                              | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1
                              | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
    HAL_Delay(120);
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
