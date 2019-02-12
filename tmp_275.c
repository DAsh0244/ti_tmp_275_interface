/*
 * tmp_275.c
 *
 *  Created on: Jan 21, 2019
 *      Author: Danyal
 */

#include "tmp_275.h"
#include "driverlib.h"

#ifdef DEBUG
#define I2C_MASTER_TX_CONFIG (I2C_MASTER_SEND_MODE | I2C_MDR_STP |      \
                              I2C_MDR_FREE | I2C_MDR_STT | I2C_MDR_IRS )

#define I2C_MASTER_RX_CONFIG (I2C_MASTER_RECEIVE_MODE | I2C_MDR_STP | \
                              I2C_MDR_FREE | I2C_MDR_STT | I2C_MDR_IRS )
#else

#define I2C_MASTER_TX_CONFIG (I2C_MASTER_SEND_MODE | I2C_MDR_STP | I2C_MDR_STT | I2C_MDR_IRS)

#define I2C_MASTER_RX_CONFIG (I2C_MASTER_RECEIVE_MODE | I2C_MDR_STP | I2C_MDR_STT | I2C_MDR_IRS )

#endif

/* basics */

/*
 * Read contents of a register from tmp275
 * master sends 2 frames:
 * 1. slave address
 * 2. register pointer
 * slave sends 3 frames:
 * 1. slave address
 * 2. data byte 1 MSB
 * 3. data byte 2
 */
uint16_t tmp_275_read_byte(TMP275_I2C_MSG* msg)
{
    uint32_t i2c_base = msg->i2c_module;
    uint16_t i2c_slave = msg->slave_addr;

    /*
     * Wait until the STP bit is cleared from any previous master
     * communication. Clearing of this bit by the module is delayed until after
     * the SCD bit is set. If this bit is not checked prior to initiating a
     * new message, the I2C could get confused.
     */
    if(I2C_getStopConditionStatus(i2c_base))
    {
        return(TMP275_MSG_ERROR_STOP_NOT_READY);
    }

    I2C_setSlaveAddress(i2c_base, i2c_slave);

    /* Check if bus busy */
    if(I2C_isBusBusy(i2c_base))
    {
        return(TMP275_MSG_ERROR_BUS_BUSY);
    }

    /* Setup number of bytes to send message_buffer and register address */
    I2C_setDataCount(i2c_base, 1);

    /* Setup data to send */
    I2C_putData(i2c_base, msg->reg_ptr);
    I2C_setConfig(i2c_base, I2C_MASTER_TX_CONFIG);
//    I2C_clearStatus(i2c_base, I2C_STS_NO_ACK);

    while (I2C_getStopConditionStatus (i2c_base) != 0 );

    /* re send slave address and start condition to initiate register read back */
    I2C_setDataCount(i2c_base, 1);
    I2C_setConfig(i2c_base, I2C_MASTER_RX_CONFIG);

    msg->reg_val = I2C_getData(i2c_base);

    I2C_sendStopCondition(i2c_base);
//    I2C_sendNACK(i2c_base);
//    NOP;
    I2C_clearStatus(i2c_base, I2C_STS_NO_ACK);
    return TMP275_MSG_SUCCESS;
}

/*
 * write a byte to the tmp275
 * length of 3 frames:
 * 1. slave address
 * 2. register pointer
 * 3. data byte 1
 */
uint16_t tmp_275_write_byte(TMP275_I2C_MSG* msg)
{
    uint32_t i2c_base = msg->i2c_module;
    uint16_t i2c_slave = msg->slave_addr;

    /*
     * Wait until the STP bit is cleared from any previous master
     * communication. Clearing of this bit by the module is delayed until after
     * the SCD bit is set. If this bit is not checked prior to initiating a
     * new message, the I2C could get confused.
     */
    if(I2C_getStopConditionStatus(i2c_base))
    {
        return(TMP275_MSG_ERROR_STOP_NOT_READY);
    }

    I2C_setSlaveAddress(i2c_base, i2c_slave);

    /* Check if bus busy */
    if(I2C_isBusBusy(i2c_base))
    {
        return(TMP275_MSG_ERROR_BUS_BUSY);
    }

    /* Setup number of bytes to send message_buffer and register address */
    I2C_setDataCount(i2c_base, 2);

    /* Setup data to send */
    I2C_putData(i2c_base, msg->reg_ptr);
    /* load in value, upper 8 bits then lower 8 bits */
    I2C_putData(i2c_base, ((msg->reg_val)&0x00FF));

    /* Send start as master transmitter */
    I2C_setConfig(i2c_base, I2C_MASTER_TX_CONFIG);
//    I2C_setConfig(i2c_base, I2C_MASTER_SEND_MODE);
//    I2C_sendStartCondition(i2c_base);
//    I2C_sendStopCondition(i2c_base);
//    I2C_clearStatus(I2CA_BASE, I2C_STS_NO_ACK);
    return TMP275_MSG_SUCCESS;
}


/*
 * Read contents of a register from tmp275
 * master sends 2 frames:
 * 1. slave address
 * 2. register pointer
 * slave sends 3 frames:
 * 1. slave address
 * 2. data byte 1 MSB
 * 3. data byte 2
 */
uint16_t tmp_275_read_word(TMP275_I2C_MSG* msg)
{
    uint32_t i2c_base = msg->i2c_module;
    uint16_t i2c_slave = msg->slave_addr;

    /*
     * Wait until the STP bit is cleared from any previous master
     * communication. Clearing of this bit by the module is delayed until after
     * the SCD bit is set. If this bit is not checked prior to initiating a
     * new message, the I2C could get confused.
     */
    if(I2C_getStopConditionStatus(i2c_base))
    {
        return(TMP275_MSG_ERROR_STOP_NOT_READY);
    }

    I2C_setSlaveAddress(i2c_base, i2c_slave);

    /* Check if bus busy */
    if(I2C_isBusBusy(i2c_base))
    {
        return(TMP275_MSG_ERROR_BUS_BUSY);
    }

    /* Setup number of bytes to send message_buffer and register address */
    I2C_setDataCount(i2c_base, 1);

    /* Setup data to send */
    I2C_putData(i2c_base, msg->reg_ptr);
    I2C_setConfig(i2c_base, I2C_MASTER_TX_CONFIG);
//    I2C_clearStatus(i2c_base, I2C_STS_NO_ACK);

    while (I2C_getStopConditionStatus (i2c_base) != 0 );

    /* resend slave address and start condition to initiate register read back */
    I2C_setDataCount(i2c_base, 2);
    I2C_setConfig(i2c_base, I2C_MASTER_RX_CONFIG);

    while (I2C_getRxFIFOStatus (i2c_base) == I2C_FIFO_RXEMPTY);
    msg->reg_val = (I2C_getData(i2c_base))<<8;
    while (I2C_getRxFIFOStatus (i2c_base) == I2C_FIFO_RXEMPTY);
    msg->reg_val |= I2C_getData(i2c_base);

    I2C_sendStopCondition(i2c_base);
//    I2C_sendNACK(i2c_base);
//    NOP;
    I2C_clearStatus(i2c_base, I2C_STS_NO_ACK);
    return TMP275_MSG_SUCCESS;
}

/*
 * write a word to the tmp275
 * length of 4 frames:
 * 1. slave address
 * 2. register pointer
 * 3. data byte 1 MSB
 * 4. data byte 2
 */
uint16_t tmp_275_write_word(TMP275_I2C_MSG* msg)
{
    uint32_t i2c_base = msg->i2c_module;
    uint16_t i2c_slave = msg->slave_addr;

    /*
     * Wait until the STP bit is cleared from any previous master
     * communication. Clearing of this bit by the module is delayed until after
     * the SCD bit is set. If this bit is not checked prior to initiating a
     * new message, the I2C could get confused.
     */
    if(I2C_getStopConditionStatus(i2c_base))
    {
        return(TMP275_MSG_ERROR_STOP_NOT_READY);
    }

    I2C_setSlaveAddress(i2c_base, i2c_slave);

    /* Check if bus busy */
    if(I2C_isBusBusy(i2c_base))
    {
        return(TMP275_MSG_ERROR_BUS_BUSY);
    }

    /* Setup number of bytes to send message_buffer and register address */
    I2C_setDataCount(i2c_base, TMP275_WRITE_FRAMES);

    /* Setup data to send */
    I2C_putData(i2c_base, msg->reg_ptr);
    /* load in value, upper 8 bits then lower 8 bits */
    I2C_putData(i2c_base, (((msg->reg_val)&0xFF00)>>8));
    I2C_putData(i2c_base, ((msg->reg_val)&0x00FF));

    /* Send start as master transmitter */
    I2C_setConfig(i2c_base, I2C_MASTER_TX_CONFIG);
//    I2C_setConfig(i2c_base, I2C_MASTER_SEND_MODE);
//    I2C_sendStartCondition(i2c_base);
//    I2C_sendStopCondition(i2c_base);
//    I2C_clearStatus(I2CA_BASE, I2C_STS_NO_ACK);
    return TMP275_MSG_SUCCESS;
}

/* utility */

static inline numeric c2f(const numeric deg_c)
{
    return (deg_c*9.0/5.0)+32.0;
}

static inline numeric f2c(const numeric deg_f)
{
    return (deg_f-32.0)*5.0/9.0;
}

uint16_t tmp2775_c2bin(const numeric deg_c)
{
    const int16_t res = (int16_t)(deg_c/TEMP_PRECISION_C);
    return res;
}

uint16_t tmp275_f2bin(const numeric deg_f)
{
    int16_t res = (int16_t)(f2c(deg_f)/TEMP_PRECISION_C);
    return res;
}

numeric tmp275_bin2c(const uint16_t temp_code)
{
    numeric res = (numeric)(temp_code*TEMP_PRECISION_C);
    return res;
}

numeric tmp275_bin2f(const uint16_t temp_code)
{
    numeric res = (numeric)(temp_code*TEMP_PRECISION_C);
    return c2f(res);
}


/*
 * determines conversion time based from device bits ratio
 * from table 6 of TMP275 datasheet: http://www.ti.com/lit/ds/symlink/tmp275.pdf
 *
 */
uint16_t tmp275_conversion_time(TMP275* dev)
{
    uint16_t status = (dev->config && 0x60)>>5;
    uint16_t conv_time = 0;
    switch(status) // mask off bits of interest
    {
    case 0:
        conv_time = 28; // 27.5ms -> 28ms
        break;
    case 1:
        conv_time = 55;
        break;
    case 2:
        conv_time = 110;
        break;
    case 3:
        conv_time = 220;
        break;
    default:
        conv_time = 0;
    }
    return conv_time;
}

/* convenience */

/*
 * set configuration register settings
 */
uint16_t tmp275_set_config(TMP275* dev, uint_fast8_t config)
{
    uint16_t status;
    dev->config = config;
    TMP275_I2C_MSG config_msg = {
        .i2c_module=dev->i2c_module,
        .slave_addr=dev->slave_addr,
        .reg_ptr=TMP275_CONFIG_REG,
        .reg_val=(uint16_t)(config),
        .status=TMP275_MSG_STATUS_SEND_WITHSTOP
    };
    do
    {
        status = tmp_275_write_byte(&config_msg);
    }
    while(status != TMP275_MSG_SUCCESS);
    return status;
}

/*
 * set low temp threshold register value
 */
uint16_t tmp275_set_tlow(TMP275* dev, int16_t val)
{
    uint16_t status;
    TMP275_I2C_MSG tlow_msg = {
        .i2c_module=dev->i2c_module,
        .slave_addr=dev->slave_addr,
        .reg_ptr=TMP275_TLOW_REG,
        .reg_val=val,
        .status=TMP275_MSG_STATUS_SEND_WITHSTOP
    };
    do
    {
        status = tmp_275_write_word(&tlow_msg);
    }
    while(status != TMP275_MSG_SUCCESS);
    return status;
}

/*
 * set high temp threshold register value
 */
uint16_t tmp275_set_thigh(TMP275* dev, int16_t val)
{
    uint16_t status;
    TMP275_I2C_MSG thigh_msg = {
        .i2c_module=dev->i2c_module,
        .slave_addr=dev->slave_addr,
        .reg_ptr=TMP275_THIGH_REG,
        .reg_val=val,
        .status=TMP275_MSG_STATUS_SEND_WITHSTOP
    };
    do
    {
        status = tmp_275_write_word(&thigh_msg);
    }
    while(status != TMP275_MSG_SUCCESS);
    return status;
}

numeric tmp275_get_temp_c(TMP275* dev)
{
    uint16_t status;
    TMP275_I2C_MSG read_temp_msg = {
        .status = TMP275_MSG_STATUS_SEND_NOSTOP,
        .slave_addr = dev->slave_addr,
        .i2c_module=dev->i2c_module,
        .reg_ptr = TMP275_TEMP_REG
    };

    do
    {
        status = (tmp_275_read_word(&read_temp_msg));
    }
    while(status != TMP275_MSG_SUCCESS);
    numeric result = tmp275_bin2c(read_temp_msg.reg_val>>4); // only upper 12 bits, right shift preserves sign
    return result;
}

numeric tmp275_get_temp_c_average(TMP275* dev, const uint_fast8_t num_averages)
{
    uint_fast8_t i;
    numeric result = 0.0f;
    for (i=0;i<num_averages;i++)
    {
        result += tmp275_get_temp_c(dev);
    }
    result /= num_averages;
    return result;
}

numeric tmp275_get_temp_f(TMP275* dev)
{
    return c2f(tmp275_get_temp_c(dev));
}

numeric tmp275_get_temp_average_f(TMP275* dev, const uint_fast8_t num_averages)
{
    return c2f(tmp275_get_temp_c_average(dev, num_averages));
}
