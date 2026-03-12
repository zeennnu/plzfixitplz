/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : DS28E18 downstream I2C pull-strength comparison test
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

#include <string.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  uint8_t result;
  uint8_t st;
  uint8_t ppd;
  uint8_t sd;
  uint8_t ll;
  uint8_t owss;
} ow_reset_info_t;

/* Private define ------------------------------------------------------------*/
#define DS2485_ADDR_7BIT              0x40U
#define DS2485_ADDR_8BIT              (DS2485_ADDR_7BIT << 1)

#define DS2485_CMD_READ_CFG           0x52U
#define DS2485_CMD_WRITE_CFG          0x99U
#define DS2485_CMD_1WIRE_SCRIPT       0x88U
#define DS2485_CMD_1WIRE_BLOCK        0xABU
#define DS2485_CMD_FULL_OW_SEQ        0x57U

#define DS2485_REG_MASTER_CFG         0x00U
#define DS2485_REG_tRSTL              0x01U
#define DS2485_REG_tMSI               0x02U
#define DS2485_REG_tMSP               0x03U
#define DS2485_REG_tRSTH              0x04U
#define DS2485_REG_PDSLEW             0x10U
#define DS2485_REG_RPUP_BUF           0x11U

#define OW_BLOCK_FLAG_RESET           0x01U
#define OW_ROM_CMD_READ_ROM           0x33U

#define DS28E18_CMD_WRITE_SEQ         0x11U
#define DS28E18_CMD_READ_SEQ          0x22U
#define DS28E18_CMD_RUN_SEQ           0x33U
#define DS28E18_CMD_DEVICE_STATUS     0x7AU
#define DS28E18_CMD_WRITE_GPIO_CFG    0x83U
#define DS28E18_CMD_READ_GPIO_CFG     0x7CU

#define DS28E18_CFG_TARGET_GPIO_CTRL  0x0BU
#define DS28E18_CFG_MODULE_GPIO       0x03U

/* pull configuration under test */
#define DS28E18_GPIO_MODE_25K_ALL     0x0FU
#define DS28E18_GPIO_MODE_2P7K_ALL    0x5FU
/* low nibble stays same style for all four lines */
#define DS28E18_GPIO_LO_COMMON        0x0FU

#define DS28E18_SEQ_ADDR0             0x000U

#define DS28E18_SEQ_I2C_START         0x02U
#define DS28E18_SEQ_I2C_STOP          0x03U
#define DS28E18_SEQ_I2C_WRITE         0xE3U
#define DS28E18_SEQ_I2C_READ_NACK     0xD3U

#define LSM6DSO_WHO_AM_I_REG          0x0FU
#define LSM6DSO_ADDR0_7BIT            0x6AU
#define LSM6DSO_ADDR1_7BIT            0x6BU

#define APP_BUF_MAX                   192U
#define DEFAULT_DELAY_STEPS           5U

/* Private variables ---------------------------------------------------------*/
static uint32_t g_loop_count = 0U;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

static void uart_write(const char *s);
static void uart_write_line(const char *s);
static void uart_write_hex8(uint8_t value);
static void uart_write_hex16(uint16_t value);
static void uart_dump_bytes(const char *label, const uint8_t *buf, uint16_t len);

static HAL_StatusTypeDef ds2485_is_ready(void);
static HAL_StatusTypeDef ds2485_tx_then_rx_poll(const uint8_t *tx, uint16_t tx_len,
                                                uint8_t *rx, uint16_t rx_len,
                                                uint32_t rx_timeout_ms);
static HAL_StatusTypeDef ds2485_read_cfg_u16(uint8_t reg, uint16_t *value, uint8_t *result);
static HAL_StatusTypeDef ds2485_write_cfg_u16(uint8_t reg, uint16_t value, uint8_t *result);
static HAL_StatusTypeDef ds2485_script_ow_reset(uint8_t ignore_presence, ow_reset_info_t *info);
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

static uint8_t app_init_and_config(void);
static uint8_t app_presence_check(void);
static uint8_t app_read_rom(uint8_t rom_out[8]);
static uint8_t app_ds28e18_write_gpio_cfg_raw(const uint8_t rom[8], uint8_t hi, uint8_t lo, uint8_t ignore_result);
static uint8_t app_ds28e18_read_gpio_cfg(const uint8_t rom[8], uint8_t *hi_out, uint8_t *lo_out);
static uint8_t app_ds28e18_device_status(const uint8_t rom[8], uint8_t *status_out);

static uint8_t app_ds28e18_write_seq(const uint8_t rom[8],
                                     uint16_t addr,
                                     const uint8_t *data,
                                     uint8_t data_len);
static uint8_t app_ds28e18_run_seq(const uint8_t rom[8],
                                   uint16_t addr,
                                   uint16_t slen,
                                   uint8_t *result_byte,
                                   uint16_t *nack_offset);
static uint8_t app_ds28e18_read_seq(const uint8_t rom[8],
                                    uint16_t addr,
                                    uint8_t slen,
                                    uint8_t *data_out,
                                    uint8_t *out_count);

static uint8_t build_addr_only_probe(uint8_t *seq, uint8_t dev_addr_7bit, uint8_t *seq_len);
static uint8_t build_whoami_probe(uint8_t *seq, uint8_t dev_addr_7bit, uint8_t *seq_len, uint8_t *read_offset);

static void run_addr_probe(const uint8_t rom[8], const char *label, uint8_t addr7);
static void run_whoami_probe(const uint8_t rom[8], const char *label, uint8_t addr7);
static void run_suite_for_pull_mode(const uint8_t rom[8], const char *label, uint8_t hi_cfg, uint8_t lo_cfg);

/* Private user code ---------------------------------------------------------*/
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

static HAL_StatusTypeDef ds2485_is_ready(void)
{
  return HAL_I2C_IsDeviceReady(&hi2c1, DS2485_ADDR_8BIT, 3, 100);
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

static HAL_StatusTypeDef ds2485_read_cfg_u16(uint8_t reg, uint16_t *value, uint8_t *result)
{
  uint8_t tx[3], rx[4];

  tx[0] = DS2485_CMD_READ_CFG;
  tx[1] = 0x01;
  tx[2] = reg;

  if (ds2485_tx_then_rx_poll(tx, sizeof(tx), rx, sizeof(rx), 10) != HAL_OK) return HAL_ERROR;
  if (result) *result = rx[1];
  if (value)  *value  = ((uint16_t)rx[3] << 8) | rx[2];
  return HAL_OK;
}

static HAL_StatusTypeDef ds2485_write_cfg_u16(uint8_t reg, uint16_t value, uint8_t *result)
{
  uint8_t tx[5], rx[2];

  tx[0] = DS2485_CMD_WRITE_CFG;
  tx[1] = 0x03;
  tx[2] = reg;
  tx[3] = (uint8_t)(value & 0xFFU);
  tx[4] = (uint8_t)((value >> 8) & 0xFFU);

  if (ds2485_tx_then_rx_poll(tx, sizeof(tx), rx, sizeof(rx), 10) != HAL_OK) return HAL_ERROR;
  if (result) *result = rx[1];
  return HAL_OK;
}

static HAL_StatusTypeDef ds2485_script_ow_reset(uint8_t ignore_presence, ow_reset_info_t *info)
{
  uint8_t tx[4], rx[4];

  tx[0] = DS2485_CMD_1WIRE_SCRIPT;
  tx[1] = 0x02;
  tx[2] = 0x00;
  tx[3] = ignore_presence ? 0x82U : 0x80U;

  if (ds2485_tx_then_rx_poll(tx, sizeof(tx), rx, sizeof(rx), 20) != HAL_OK) return HAL_ERROR;

  if (info)
  {
    info->result = rx[1];
    info->st     = rx[3];
    info->ppd    = (uint8_t)((rx[3] >> 1) & 0x01U);
    info->sd     = (uint8_t)((rx[3] >> 2) & 0x01U);
    info->ll     = (uint8_t)((rx[3] >> 3) & 0x01U);
    info->owss   = (uint8_t)((rx[3] >> 4) & 0x01U);
  }
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

  tx[0] = DS2485_CMD_FULL_OW_SEQ;
  tx[1] = (uint8_t)(1U + 8U + ow_data_len);
  tx[2] = delay_steps;
  memcpy(&tx[3], rom_id, 8U);
  memcpy(&tx[11], ow_data, ow_data_len);

  return ds2485_tx_then_rx_poll(tx, (uint16_t)(11U + ow_data_len), rx, rx_len, rx_timeout_ms);
}

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

static uint8_t app_init_and_config(void)
{
  uint8_t result;

  uart_write("[1] Probing DS2485 ... ");
  if (ds2485_is_ready() != HAL_OK)
  {
    uart_write_line("FAIL");
    return 0U;
  }
  uart_write_line("ACK OK");

  (void)ds2485_write_cfg_u16(DS2485_REG_RPUP_BUF, 0x8026U, &result);
  (void)ds2485_write_cfg_u16(DS2485_REG_MASTER_CFG, 0x1000U, &result);
  (void)ds2485_write_cfg_u16(DS2485_REG_PDSLEW, 0x0006U, &result);
  (void)ds2485_write_cfg_u16(DS2485_REG_tRSTL, 0xA080U, &result);
  (void)ds2485_write_cfg_u16(DS2485_REG_tMSI,  0x8060U, &result);
  (void)ds2485_write_cfg_u16(DS2485_REG_tMSP,  0x8460U, &result);
  (void)ds2485_write_cfg_u16(DS2485_REG_tRSTH, 0xA080U, &result);

  uart_write_line("[2] DS2485 config write done");
  return 1U;
}

static uint8_t app_presence_check(void)
{
  ow_reset_info_t info;

  uart_write("[3] Presence check ... ");
  if (ds2485_script_ow_reset(0U, &info) != HAL_OK)
  {
    uart_write_line("I2C FAIL");
    return 0U;
  }

  uart_write("result=0x"); uart_write_hex8(info.result);
  uart_write(" ST=0x"); uart_write_hex8(info.st);
  uart_write(" PPD="); uart_write(info.ppd ? "1" : "0");
  uart_write(" SD=");  uart_write(info.sd ? "1" : "0");
  uart_write(" LL=");  uart_write(info.ll ? "1" : "0");
  uart_write("\r\n");

  return ((info.result == 0xAAU) && (info.ppd == 1U) && (info.sd == 0U)) ? 1U : 0U;
}

static uint8_t app_read_rom(uint8_t rom_out[8])
{
  uint8_t ow_tx[9], rx[11], i;

  ow_tx[0] = OW_ROM_CMD_READ_ROM;
  for (i = 1U; i < sizeof(ow_tx); i++) ow_tx[i] = 0xFFU;

  memset(rx, 0xFF, sizeof(rx));
  if (ds2485_1wire_block(OW_BLOCK_FLAG_RESET, ow_tx, (uint8_t)sizeof(ow_tx), rx, sizeof(rx), 60) != HAL_OK)
  {
    uart_write_line("[ROM] RX FAIL");
    return 0U;
  }

  uart_write("[ROM] len=0x"); uart_write_hex8(rx[0]);
  uart_write(" result=0x"); uart_write_hex8(rx[1]); uart_write("\r\n");
  uart_dump_bytes("[ROM] raw   = ", rx, sizeof(rx));

  if (rx[1] != 0xAAU) return 0U;

  memcpy(rom_out, &rx[3], 8U);
  uart_dump_bytes("[ROM] id    = ", rom_out, 8U);

  if (crc8_maxim(rom_out, 7U) != rom_out[7])
  {
    uart_write_line("[ROM] CRC FAIL");
    return 0U;
  }
  uart_write_line("[ROM] CRC OK");
  return 1U;
}

static uint8_t app_ds28e18_write_gpio_cfg_raw(const uint8_t rom[8], uint8_t hi, uint8_t lo, uint8_t ignore_result)
{
  uint8_t ow_data[5] = {
    DS28E18_CMD_WRITE_GPIO_CFG,
    DS28E18_CFG_TARGET_GPIO_CTRL,
    DS28E18_CFG_MODULE_GPIO,
    hi,
    lo
  };
  uint8_t rx[4];

  memset(rx, 0xFF, sizeof(rx));
  if (ds2485_full_ow_sequence(DEFAULT_DELAY_STEPS, rom, ow_data, sizeof(ow_data), rx, sizeof(rx), 120) != HAL_OK)
  {
    uart_write_line("[GPIO-W] RX FAIL");
    return 0U;
  }

  uart_write("[GPIO-W] hi=0x"); uart_write_hex8(hi);
  uart_write(" lo=0x"); uart_write_hex8(lo);
  uart_write(" len=0x"); uart_write_hex8(rx[0]);
  uart_write(" ds2485=0x"); uart_write_hex8(rx[1]);
  uart_write(" ow_len=0x"); uart_write_hex8(rx[2]);
  uart_write(" ow_res=0x"); uart_write_hex8(rx[3]);
  uart_write("\r\n");

  if (ignore_result) return (rx[1] == 0xAAU) ? 1U : 0U;
  return ((rx[1] == 0xAAU) && (rx[2] == 0x01U) && (rx[3] == 0xAAU)) ? 1U : 0U;
}

static uint8_t app_ds28e18_read_gpio_cfg(const uint8_t rom[8], uint8_t *hi_out, uint8_t *lo_out)
{
  uint8_t ow_data[3] = {
    DS28E18_CMD_READ_GPIO_CFG,
    DS28E18_CFG_TARGET_GPIO_CTRL,
    DS28E18_CFG_MODULE_GPIO
  };
  uint8_t rx[6];

  memset(rx, 0xFF, sizeof(rx));
  if (ds2485_full_ow_sequence(DEFAULT_DELAY_STEPS, rom, ow_data, sizeof(ow_data), rx, sizeof(rx), 120) != HAL_OK)
  {
    uart_write_line("[GPIO-R] RX FAIL");
    return 0U;
  }

  uart_write("[GPIO-R] len=0x"); uart_write_hex8(rx[0]);
  uart_write(" ds2485=0x"); uart_write_hex8(rx[1]);
  uart_write(" ow_len=0x"); uart_write_hex8(rx[2]);
  uart_write(" ow_res=0x"); uart_write_hex8(rx[3]);
  uart_write(" hi=0x"); uart_write_hex8(rx[4]);
  uart_write(" lo=0x"); uart_write_hex8(rx[5]);
  uart_write("\r\n");

  if ((rx[1] == 0xAAU) && (rx[2] == 0x03U) && (rx[3] == 0xAAU))
  {
    if (hi_out) *hi_out = rx[4];
    if (lo_out) *lo_out = rx[5];
    return 1U;
  }
  return 0U;
}

static uint8_t app_ds28e18_device_status(const uint8_t rom[8], uint8_t *status_out)
{
  uint8_t ow_data[1] = { DS28E18_CMD_DEVICE_STATUS };
  uint8_t rx[8];

  memset(rx, 0xFF, sizeof(rx));
  if (ds2485_full_ow_sequence(DEFAULT_DELAY_STEPS, rom, ow_data, sizeof(ow_data), rx, sizeof(rx), 120) != HAL_OK)
  {
    uart_write_line("[STAT ] RX FAIL");
    return 0U;
  }

  uart_write("[STAT ] len=0x"); uart_write_hex8(rx[0]);
  uart_write(" ds2485=0x"); uart_write_hex8(rx[1]);
  uart_write(" ow_len=0x"); uart_write_hex8(rx[2]);
  uart_write(" ow_res=0x"); uart_write_hex8(rx[3]);
  uart_write(" st=0x"); uart_write_hex8(rx[4]);
  uart_write("\r\n");

  if ((rx[1] == 0xAAU) && (rx[2] == 0x05U) && (rx[3] == 0xAAU))
  {
    if (status_out) *status_out = rx[4];
    return 1U;
  }
  return 0U;
}

static uint8_t app_ds28e18_write_seq(const uint8_t rom[8],
                                     uint16_t addr,
                                     const uint8_t *data,
                                     uint8_t data_len)
{
  uint8_t ow_data[3 + 128];
  uint8_t rx[4];

  if ((data_len == 0U) || (data_len > 128U)) return 0U;

  ow_data[0] = DS28E18_CMD_WRITE_SEQ;
  ow_data[1] = (uint8_t)(addr & 0xFFU);
  ow_data[2] = (uint8_t)((addr >> 8) & 0x01U);
  memcpy(&ow_data[3], data, data_len);

  memset(rx, 0xFF, sizeof(rx));
  if (ds2485_full_ow_sequence(DEFAULT_DELAY_STEPS, rom, ow_data, (uint8_t)(3U + data_len), rx, sizeof(rx), 150) != HAL_OK)
  {
    uart_write_line("[SEQ-W] RX FAIL");
    return 0U;
  }

  uart_write("[SEQ-W] len=0x"); uart_write_hex8(rx[0]);
  uart_write(" ds2485=0x"); uart_write_hex8(rx[1]);
  uart_write(" ow_len=0x"); uart_write_hex8(rx[2]);
  uart_write(" ow_res=0x"); uart_write_hex8(rx[3]);
  uart_write("\r\n");

  return ((rx[1] == 0xAAU) && (rx[2] == 0x01U) && (rx[3] == 0xAAU)) ? 1U : 0U;
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
  ow_data[3] = (uint8_t)((slen >> 8) & 0x01U);

  memset(rx, 0xFF, sizeof(rx));
  if (ds2485_full_ow_sequence(DEFAULT_DELAY_STEPS, rom, ow_data, sizeof(ow_data), rx, sizeof(rx), 200) != HAL_OK)
  {
    uart_write_line("[SEQ-RUN] RX FAIL");
    return 0U;
  }

  uart_write("[SEQ-RUN] len=0x"); uart_write_hex8(rx[0]);
  uart_write(" ds2485=0x"); uart_write_hex8(rx[1]);
  uart_write(" ow_len=0x"); uart_write_hex8(rx[2]);
  uart_write(" ow_res=0x"); uart_write_hex8(rx[3]);
  if (rx[2] >= 3U)
  {
    uart_write(" snack_lo=0x"); uart_write_hex8(rx[4]);
    uart_write(" snack_hi=0x"); uart_write_hex8(rx[5]);
  }
  uart_write("\r\n");

  if (result_byte) *result_byte = rx[3];
  if ((rx[1] == 0xAAU) && (rx[2] == 3U) && (rx[3] == 0x88U) && nack_offset)
    *nack_offset = ((uint16_t)rx[5] << 8) | rx[4];

  return (rx[1] == 0xAAU) ? 1U : 0U;
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

  rx_len = (uint16_t)(4U + slen);

  memset(rx, 0xFF, sizeof(rx));
  if (ds2485_full_ow_sequence(DEFAULT_DELAY_STEPS, rom, ow_data, sizeof(ow_data), rx, rx_len, 200) != HAL_OK)
  {
    uart_write_line("[SEQ-RD ] RX FAIL");
    return 0U;
  }

  uart_write("[SEQ-RD ] len=0x"); uart_write_hex8(rx[0]);
  uart_write(" ds2485=0x"); uart_write_hex8(rx[1]);
  uart_write(" ow_len=0x"); uart_write_hex8(rx[2]);
  uart_write(" ow_res=0x"); uart_write_hex8(rx[3]);
  uart_write("\r\n");

  payload_count = (rx[2] >= 1U) ? (uint8_t)(rx[2] - 1U) : 0U;

  if (payload_count > slen) payload_count = slen;

  if ((rx[1] == 0xAAU) && (rx[3] == 0xAAU))
  {
    if (data_out && payload_count > 0U) memcpy(data_out, &rx[4], payload_count);
    if (out_count) *out_count = payload_count;
    return 1U;
  }
  return 0U;
}

static uint8_t build_addr_only_probe(uint8_t *seq, uint8_t dev_addr_7bit, uint8_t *seq_len)
{
  uint8_t idx = 0U;
  uint8_t addr_w = (uint8_t)(dev_addr_7bit << 1);

  if (!seq || !seq_len) return 0U;

  seq[idx++] = DS28E18_SEQ_I2C_START;
  seq[idx++] = DS28E18_SEQ_I2C_WRITE;
  seq[idx++] = 0x01U;
  seq[idx++] = addr_w;
  seq[idx++] = DS28E18_SEQ_I2C_STOP;

  *seq_len = idx;
  return 1U;
}

static uint8_t build_whoami_probe(uint8_t *seq, uint8_t dev_addr_7bit, uint8_t *seq_len, uint8_t *read_offset)
{
  uint8_t idx = 0U;
  uint8_t addr_w = (uint8_t)(dev_addr_7bit << 1);
  uint8_t addr_r = (uint8_t)((dev_addr_7bit << 1) | 0x01U);

  if (!seq || !seq_len || !read_offset) return 0U;

  seq[idx++] = DS28E18_SEQ_I2C_START;
  seq[idx++] = DS28E18_SEQ_I2C_WRITE;
  seq[idx++] = 0x02U;
  seq[idx++] = addr_w;
  seq[idx++] = LSM6DSO_WHO_AM_I_REG;
  seq[idx++] = DS28E18_SEQ_I2C_START;
  seq[idx++] = DS28E18_SEQ_I2C_WRITE;
  seq[idx++] = 0x01U;
  seq[idx++] = addr_r;
  seq[idx++] = DS28E18_SEQ_I2C_READ_NACK;
  seq[idx++] = 0x01U;
  *read_offset = idx;
  seq[idx++] = 0xFFU;
  seq[idx++] = DS28E18_SEQ_I2C_STOP;

  *seq_len = idx;
  return 1U;
}

static void run_addr_probe(const uint8_t rom[8], const char *label, uint8_t addr7)
{
  uint8_t seq[16];
  uint8_t seq_len = 0U;
  uint8_t run_res = 0xFFU;
  uint16_t nack_off = 0xFFFFU;
  uint8_t dump[16];
  uint8_t dump_len = 0U;

  uart_write("[ADDR] "); uart_write(label);
  uart_write(" target=0x"); uart_write_hex8(addr7); uart_write("\r\n");

  if (!build_addr_only_probe(seq, addr7, &seq_len))
  {
    uart_write_line("[ADDR] build fail");
    return;
  }

  uart_dump_bytes("[ADDR] seq   = ", seq, seq_len);

  if (!app_ds28e18_write_seq(rom, DS28E18_SEQ_ADDR0, seq, seq_len))
  {
    uart_write_line("[ADDR] write seq fail");
    return;
  }

  if (!app_ds28e18_run_seq(rom, DS28E18_SEQ_ADDR0, seq_len, &run_res, &nack_off))
  {
    uart_write_line("[ADDR] run transport fail");
    return;
  }

  if (run_res == 0x88U)
  {
    uart_write("[ADDR] NACK offset=0x"); uart_write_hex16(nack_off); uart_write("\r\n");
  }
  else
  {
    uart_write("[ADDR] run result=0x"); uart_write_hex8(run_res); uart_write("\r\n");
  }

  if (app_ds28e18_read_seq(rom, DS28E18_SEQ_ADDR0, seq_len, dump, &dump_len))
    uart_dump_bytes("[ADDR] SRAM  = ", dump, dump_len);
}

static void run_whoami_probe(const uint8_t rom[8], const char *label, uint8_t addr7)
{
  uint8_t seq[32];
  uint8_t seq_len = 0U;
  uint8_t read_offset = 0U;
  uint8_t run_res = 0xFFU;
  uint16_t nack_off = 0xFFFFU;
  uint8_t dump[32];
  uint8_t dump_len = 0U;

  uart_write("[WHO ] "); uart_write(label);
  uart_write(" target=0x"); uart_write_hex8(addr7); uart_write("\r\n");

  if (!build_whoami_probe(seq, addr7, &seq_len, &read_offset))
  {
    uart_write_line("[WHO ] build fail");
    return;
  }

  uart_dump_bytes("[WHO ] seq   = ", seq, seq_len);

  if (!app_ds28e18_write_seq(rom, DS28E18_SEQ_ADDR0, seq, seq_len))
  {
    uart_write_line("[WHO ] write seq fail");
    return;
  }

  if (!app_ds28e18_run_seq(rom, DS28E18_SEQ_ADDR0, seq_len, &run_res, &nack_off))
  {
    uart_write_line("[WHO ] run transport fail");
    return;
  }

  if (run_res == 0x88U)
  {
    uart_write("[WHO ] NACK offset=0x"); uart_write_hex16(nack_off); uart_write("\r\n");
  }
  else
  {
    uart_write("[WHO ] run result=0x"); uart_write_hex8(run_res); uart_write("\r\n");
  }

  if (app_ds28e18_read_seq(rom, DS28E18_SEQ_ADDR0, seq_len, dump, &dump_len))
  {
    uart_dump_bytes("[WHO ] SRAM  = ", dump, dump_len);
    if (read_offset < dump_len)
    {
      uart_write("[WHO ] val   = 0x"); uart_write_hex8(dump[read_offset]); uart_write("\r\n");
    }
  }
}

static void run_suite_for_pull_mode(const uint8_t rom[8], const char *label, uint8_t hi_cfg, uint8_t lo_cfg)
{
  uint8_t rb_hi = 0U, rb_lo = 0U;

  uart_write_line("--------------------------------");
  uart_write("[SUIT] "); uart_write_line(label);

  if (!app_ds28e18_write_gpio_cfg_raw(rom, hi_cfg, lo_cfg, 0U))
  {
    uart_write_line("[SUIT] GPIO cfg write fail");
    return;
  }

  if (app_ds28e18_read_gpio_cfg(rom, &rb_hi, &rb_lo))
  {
    uart_write("[SUIT] GPIO rb hi=0x"); uart_write_hex8(rb_hi);
    uart_write(" lo=0x"); uart_write_hex8(rb_lo); uart_write("\r\n");
  }
  else
  {
    uart_write_line("[SUIT] GPIO cfg read fail");
  }

  run_addr_probe(rom, label, LSM6DSO_ADDR0_7BIT);
  HAL_Delay(20);
  run_addr_probe(rom, label, LSM6DSO_ADDR1_7BIT);
  HAL_Delay(20);

  run_whoami_probe(rom, label, LSM6DSO_ADDR0_7BIT);
  HAL_Delay(20);
  run_whoami_probe(rom, label, LSM6DSO_ADDR1_7BIT);
  HAL_Delay(20);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  uint8_t rom0[8], rom1[8];
  uint8_t st = 0U;
  uint8_t i;
  uint8_t same_rom = 1U;

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  HAL_Delay(100);

  uart_write_line("================================");
  uart_write_line(" UART OK - PULL MODE A/B TEST");
  uart_write_line("================================");

  if (!app_init_and_config())
  {
    uart_write_line("[ERR] init fail");
    while (1) { HAL_Delay(500); }
  }

  if (!app_presence_check())
  {
    uart_write_line("[ERR] presence fail");
    while (1) { HAL_Delay(500); }
  }

  uart_write_line("--------------------------------");
  uart_write_line("[4] READ ROM before initial GPIO write");
  if (!app_read_rom(rom0))
  {
    uart_write_line("[ERR] first ROM fail");
    while (1) { HAL_Delay(500); }
  }

  uart_write_line("--------------------------------");
  uart_write_line("[5] First WRITE GPIO CONFIG");
  (void)app_ds28e18_write_gpio_cfg_raw(rom0, DS28E18_GPIO_MODE_25K_ALL, DS28E18_GPIO_LO_COMMON, 1U);
  HAL_Delay(20);

  uart_write_line("--------------------------------");
  uart_write_line("[6] READ ROM after initial GPIO write");
  if (!app_read_rom(rom1))
  {
    uart_write_line("[ERR] second ROM fail");
    while (1) { HAL_Delay(500); }
  }

  for (i = 0U; i < 8U; i++)
  {
    if (rom0[i] != rom1[i]) { same_rom = 0U; break; }
  }
  uart_write_line(same_rom ? "[ROM ] unchanged" : "[ROM ] unique ROM populated");

  uart_write_line("--------------------------------");
  uart_write_line("[7] DEVICE STATUS check");
  (void)app_ds28e18_device_status(rom1, &st);
  (void)app_ds28e18_device_status(rom1, &st);

  /* Compare pull strengths */
  run_suite_for_pull_mode(rom1, "25k pull-up mode",  DS28E18_GPIO_MODE_25K_ALL,  DS28E18_GPIO_LO_COMMON);
  run_suite_for_pull_mode(rom1, "2.7k pull-up mode", DS28E18_GPIO_MODE_2P7K_ALL, DS28E18_GPIO_LO_COMMON);

  uart_write_line("--------------------------------");
  uart_write_line("[DONE] Compare address ACK behaviour between 25k and 2.7k pull settings");
  uart_write_line("[8] Entering main loop...");

  while (1)
  {
    g_loop_count++;
    if ((g_loop_count % 10U) == 0U) uart_write_line("loop alive");
    HAL_Delay(500);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
