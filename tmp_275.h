/*
 * tmp_275.h
 *
 * The TMP275 supports the transmission protocol for fast (up to 400 kHz) and high-speed (up to 2.38 MHz) modes.
 * All data bytes are transmitted MSB first.
 *  Created on: Jan 17, 2019
 *      Author: Danyal Ahsanullah
 */

#ifndef TMP_275_H_
#define TMP_275_H_

#include <stdint.h>
#include <stdbool.h>
#include "flex_types.h"


#define TEMP_PRECISION_C  (0.0625)
#define TEMP_PRECISION_F  (TEMP_PRECISION_C*9.0/5.0)
/* max buffer size is  from the returned */
#define TMP275_BUF_SIZE   (3)

#define TMP275_WRITE_FRAMES (3)
#define TMP275_READ_FRAMES (2)

/* bus slave address helpers*/
#define TMP275_BASE_ADDR (0x48)

#define TMP275_FORM_ADDR(A2, A1, A0)  (TMP275_BASE_ADDR|(A2<<2)|(A1<<1)|(A0))



/* bus slave addresses */
#define TMP275_ADDR_0 TMP275_FORM_ADDR(0,0,0)
#define TMP275_ADDR_1 TMP275_FORM_ADDR(0,0,1)
#define TMP275_ADDR_2 TMP275_FORM_ADDR(0,1,0)
#define TMP275_ADDR_3 TMP275_FORM_ADDR(0,1,1)
#define TMP275_ADDR_4 TMP275_FORM_ADDR(1,0,0)
#define TMP275_ADDR_5 TMP275_FORM_ADDR(1,0,1)
#define TMP275_ADDR_6 TMP275_FORM_ADDR(1,1,0)
#define TMP275_ADDR_7 TMP275_FORM_ADDR(1,1,1)

/* device register offsets */
#define TMP275_TEMP_REG   (0x0)
#define TMP275_CONFIG_REG (0x1)
#define TMP275_TLOW_REG   (0x2)
#define TMP275_THIGH_REG  (0x3)

/*
 * configuration macros
 */

/* Shutdown mode */
#define TMP275_SD_ENABLE  (0x1)
#define TMP275_SD_DISABLE (0x0)

/* Thermostat mode */
#define TMP275_TM_ENABLE  (0x1<<1)
#define TMP275_TM_DISABLE (0x0<<1)

/* Polarity mode */
#define TMP275_POL_ENABLE  (0x1<<2)
#define TMP275_POL_DISABLE (0x0<<2)

/* Fault Queue mode */
#define TMP275_FQ_1 (0x0<<3)
#define TMP275_FQ_2 (0x1<<3)
#define TMP275_FQ_4 (0x2<<3)
#define TMP275_FQ_6 (0x3<<3)

/* resolution mode */
#define TMP275_9_BITS  (0x0<<4)
#define TMP275_10_BITS (0x1<<4)
#define TMP275_11_BITS (0x2<<4)
#define TMP275_12_BITS (0x3<<4)

/* One-Shot mode */
#define TMP275_OS_ENABLE  (0x0<<6)
#define TMP275_OS_DISABLE (0x1<<6)

/* macros for bus control */
#define TMP275_BUS_HS_ENABLE (0x0FU) // send this byte as pay load to enable High Speed bus

///* I2C message states for TMP275_I2C_MSG struct */
//#define TMP275_MSG_STATUS_INACTIVE         (0x0000) /* Message not in use, do not send */
//#define TMP275_MSG_STATUS_SEND_WITHSTOP    (0x0010) /* Send message with stop bit */
//#define TMP275_MSG_STATUS_WRITE_BUSY       (0x0011) /* Message sent, wait for stop */
//#define TMP275_MSG_STATUS_SEND_NOSTOP      (0x0020) /* Send message without stop bit */
//#define TMP275_MSG_STATUS_SEND_NOSTOP_BUSY (0x0021) /* Message sent, wait for ARDY */
//#define TMP275_MSG_STATUS_RESTART          (0x0022) /* Ready to become master-receiver */
//#define TMP275_MSG_STATUS_READ_BUSY        (0x0023) /* Wait for stop before reading data */

/* Error messages for read and write functions */
#define TMP275_MSG_ERROR_BUS_BUSY             (0x1000)
#define TMP275_MSG_ERROR_STOP_NOT_READY       (0x5555)
#define TMP275_MSG_SUCCESS                    (0x0000)

/*
 * interface API
 */

/* typedefs */

typedef enum
{
    TMP275_MSG_STATUS_INACTIVE         = 0x0000,  /*Message not in use, do not send */
    TMP275_MSG_STATUS_SEND_WITHSTOP    = 0x0010,  /*Send message with stop bit */
    TMP275_MSG_STATUS_WRITE_BUSY       = 0x0011,  /*Message sent, wait for stop */
    TMP275_MSG_STATUS_SEND_NOSTOP      = 0x0020,  /*Send message without stop bit */
    TMP275_MSG_STATUS_SEND_NOSTOP_BUSY = 0x0021,  /*Message sent, wait for ARDY */
    TMP275_MSG_STATUS_RESTART          = 0x0022,  /*Ready to become master-receiver */
    TMP275_MSG_STATUS_READ_BUSY        = 0x0023   /*Wait for stop before reading data */
} tmp275_msg_status;

/*
typedef enum
{
    TEMP_REG   = 0x0U,
    CONFIG_REG = 0x1U,
    TLOW_REG   = 0x2U,
    THIGH_REG  = 0x3U
} tmp275_register;


typedef enum
{
#define TMP275_TM_ENABLE  (0x1<<1)
#define TMP275_TM_DISABLE (0x0<<1)
}tmp275_thermostat_mode;

typedef enum
{
#define TMP275_SD_ENABLE  (0x1)
#define TMP275_SD_DISABLE (0x0)
} tmp275_shutdown_mode;

typedef enum
{
#define TMP275_9_BITS  (0x0<<4)
#define TMP275_10_BITS (0x1<<4)
#define TMP275_11_BITS (0x2<<4)
#define TMP275_12_BITS (0x3<<4)
} tmp275_resolution_mode;

typedef enum
{
#define TMP275_FQ_1 (0x0<<3)
#define TMP275_FQ_2 (0x1<<3)
#define TMP275_FQ_4 (0x2<<3)
#define TMP275_FQ_6 (0x3<<3)
}tmp275_fault_queue_mode;


*/

/* structs for data management */
typedef struct TMP275 {
    uint16_t slave_addr;  /* Slave address of device */
    uint32_t i2c_module;  /* I2C module to be sent from/Rx from */
    uint_fast8_t config; /* configuration for device */
} TMP275;

typedef struct TMP275_I2C_MSG{
    uint32_t      i2c_module;       /* I2C module to be sent from/Rx from */
    uint_fast16_t slave_addr;       /* Slave address tied to the message */
    int16_t      reg_val;          /* value to load into register - will be split upper half and lower half bits */
    uint_fast8_t  reg_ptr;          /* register address of data associated */
    tmp275_msg_status  status;           /* msg status code */
} TMP275_I2C_MSG;

/* functions */

/* basics */
uint16_t tmp_275_read_byte(TMP275_I2C_MSG* msg);

uint16_t tmp_275_write_byte(TMP275_I2C_MSG* msg);

uint16_t tmp_275_read_word(TMP275_I2C_MSG* msg);

uint16_t tmp_275_write_word(TMP275_I2C_MSG* msg);

/* utility */
uint16_t tmp2775_c2bin(const numeric deg_c);

uint16_t tmp275_f2bin(const numeric def_f);

numeric tmp275_bin2c(const uint16_t );

numeric tmp275_bin2f(const uint16_t);

uint16_t tmp275_conversion_time(TMP275* dev);

//static inline uint_fast8_t tmp275_check_resolution(TMP275* dev)
//{
//    return (9+(((dev->config)&TMP275_12_BITS)>>5));
//}

/* convenience */
uint16_t tmp275_set_config(TMP275* dev, uint_fast8_t config);

uint16_t tmp275_set_tlow(TMP275* dev, int16_t val);

uint16_t tmp275_set_thigh(TMP275* dev, int16_t val);

numeric tmp275_get_temp_c(TMP275* dev);

numeric tmp275_get_temp_c_average(TMP275* dev, const uint_fast8_t num_averages);

numeric tmp275_get_temp_f(TMP275* dev);

numeric tmp275_get_temp_average_f(TMP275* dev, const uint_fast8_t num_averages);


#endif /* TMP_275_H_ */
