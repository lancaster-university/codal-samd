#include "ZI2C.h"
#include "codal_target_hal.h"
#include "hal_gpio.h"
#include "CodalDmesg.h"

extern "C"
{
    #include "pins.h"
    #include "sercom.h"
    #include "hal_i2c_m_sync.h"
}

using namespace codal;

#define MAX_I2C_RETRIES     2

/**
 * Constructor.
 */
ZI2C::ZI2C(ZPin &sda, ZPin &scl) : codal::I2C(sda,scl), sda(sda), scl(scl)
{
    const mcu_pin_obj_t* sda_pin = samd_peripherals_get_pin(sda.name);
    const mcu_pin_obj_t* scl_pin = samd_peripherals_get_pin(scl.name);

    if (sda_pin->sercom[0].index == 0x3f || scl_pin->sercom[1].index ==  0x3f)
        target_panic(864);

    int sercomIdx = -1;
    int sda_fun = -1;
    int scl_fun = -1;

    if (sda_pin->sercom[0].pad == 0)
    {
        sda_fun = MUX_C; // c
        sercomIdx = sda_pin->sercom[0].index;
    } else if (sda_pin->sercom[1].pad == 0)
    {
        sda_fun = MUX_D; // d
        sercomIdx = sda_pin->sercom[1].index;
    } else
        target_panic(DEVICE_HARDWARE_CONFIGURATION_ERROR);

    if (scl_pin->sercom[0].index == sercomIdx)
    {
        if(scl_pin->sercom[0].pad != 1)
            target_panic(DEVICE_HARDWARE_CONFIGURATION_ERROR);

        scl_fun = MUX_C; // c
    }
    else if (scl_pin->sercom[1].index == sercomIdx)
    {
        if(scl_pin->sercom[1].pad != 0)
            target_panic(DEVICE_HARDWARE_CONFIGURATION_ERROR);

        scl_fun = MUX_D; // d
    }
    else
        target_panic(DEVICE_HARDWARE_CONFIGURATION_ERROR);

    DMESG("SDA pad %d, idx %d, fn: %d", sda_pin->sercom[0].pad, sda_pin->sercom[0].index, sda_fun);
    DMESG("SCL pad %d, idx %d, fn: %d", scl_pin->sercom[0].pad, scl_pin->sercom[0].index, scl_fun);
    DMESG("SERCOM IDX %d", sercomIdx);

    sda._setMux(sda_fun);
    scl._setMux(scl_fun);

    Sercom* i2c_sercom = sercom_insts[sercomIdx];
    samd_peripherals_sercom_clock_init(i2c_sercom, sercomIdx);
    int ret = 0;
    ret = i2c_m_sync_init(&i2c, i2c_sercom);
    DMESG("INIT ret: %d",ret);
    ret = i2c_m_sync_set_baudrate(&i2c, 0, 100); // set i2c freq to 100khz
    DMESG("baud ret: %d",ret);
    ret = i2c_m_sync_enable(&i2c);
    DMESG("en ret: %d",ret);
}

/** Set the frequency of the I2C interface
 *
 * @param frequency The bus frequency in hertz
 */
int ZI2C::setFrequency(uint32_t frequency)
{
    i2c_m_sync_disable(&i2c);
    int ret = i2c_m_sync_set_baudrate(&i2c, 0, frequency / 1000);
    i2c_m_sync_enable(&i2c);
    DMESG("SET FREQ: %d, ret: %d", frequency, ret);
    return (ret < 0) ? DEVICE_INVALID_PARAMETER : DEVICE_OK;
}

/**
* Issues a standard, I2C command write to the I2C bus.
* This consists of:
*  - Asserting a Start condition on the bus
*  - Selecting the Slave address (as an 8 bit address)
*  - Writing a number of raw data bytes provided
*  - Asserting a Stop condition on the bus
*
* The CPU will busy wait until the transmission is complete.
*
* @param address The 8bit I2C address of the device to write to
* @param data pointer to the bytes to write
* @param len the number of bytes to write
* @param repeated Suppresses the generation of a STOP condition if set. Default: false;
*
* @return DEVICE_OK on success, DEVICE_I2C_ERROR if the the write request failed.
*/
int ZI2C::write(uint16_t address, uint8_t *data, int len, bool repeated)
{
    DMESG("WRITE: %d",address);
    struct _i2c_m_msg msg;

    for (int i = 0; i < MAX_I2C_RETRIES; i++)
    {
        msg.addr = address;
        msg.len = len;
        msg.flags  = repeated ? 0 : I2C_M_STOP;
        msg.buffer = (uint8_t *) data;
        int ret = _i2c_m_sync_transfer(&i2c.device, &msg);

        if (ret == I2C_OK)
        {
            DMESG("OK");
            return DEVICE_OK;
        }
    }

    DMESG("ERR");
    return DEVICE_I2C_ERROR;
}

/**
     * Issues a standard, 2 byte I2C command read to the I2C bus.
     * This consists of:
     *  - Asserting a Start condition on the bus
     *  - Selecting the Slave address (as an 8 bit address)
     *  - reading "len" bytes of raw 8 bit data into the buffer provided
     *  - Asserting a Stop condition on the bus
     *
     * The CPU will busy wait until the transmission is complete.
     *
     * @param address The 8bit I2C address of the device to read from
     * @param data pointer to store the the bytes read
     * @param len the number of bytes to read into the buffer
     * @param repeated Suppresses the generation of a STOP condition if set. Default: false;
     *
     * @return DEVICE_OK on success, DEVICE_I2C_ERROR if the the read request failed.
     */
int ZI2C::read(uint16_t address, uint8_t *data, int len, bool repeated)
{
    DMESG("READ: %d",address);
    struct _i2c_m_msg msg;

    for (int i = 0; i < MAX_I2C_RETRIES; i++)
    {
        msg.addr   = address;
        msg.len    = len;
        msg.flags  = (repeated ? 0 : I2C_M_STOP) | I2C_M_RD;
        msg.buffer = data;
        int ret = _i2c_m_sync_transfer(&i2c.device, &msg);

        if (ret == I2C_OK)
        {
            DMESG("OK");
            return DEVICE_OK;
        }
    }

    DMESG("ERR");
    return DEVICE_I2C_ERROR;
}

/**
 * Performs a typical register write operation to the I2C slave device provided.
 * This consists of:
 *  - Asserting a Start condition on the bus
 *  - Selecting the Slave address (as an 8 bit address)
 *  - Writing the 8 bit register address provided
 *  - Writing the 8 bit value provided
 *  - Asserting a Stop condition on the bus
 *
 * The CPU will busy wait until the transmission is complete..
 *
 * @param address 8bit address of the device to write to
 * @param reg The 8bit address of the register to write to.
 * @param value The value to write.
 *
 * @return DEVICE_OK on success, DEVICE_I2C_ERROR if the the write request failed.
 */
int ZI2C::writeRegister(uint16_t address, uint8_t reg, uint8_t value)
{
    uint8_t command[2];
    command[0] = reg;
    command[1] = value;
    return write(address, command, 2, false);
}

/**
     * Performs a typical register read operation to the I2C slave device provided.
     * This consists of:
     *  - Asserting a Start condition on the bus
     *  - Selecting the Slave address (as an 8 bit address, I2C WRITE)
     *  - Selecting a RAM register address in the slave
     *  - Asserting a Stop condition on the bus
     *  - Asserting a Start condition on the bus
     *  - Selecting the Slave address (as an 8 bit address, I2C READ)
     *  - Performing an 8 bit read operation (of the requested register)
     *  - Asserting a Stop condition on the bus
     *
     * The CPU will busy wait until the transmission is complete..
     *
     * @param address 8bit I2C address of the device to read from
     * @param reg The 8bit register address of the to read.
     * @param data A pointer to a memory location to store the result of the read operation
     * @param length The number of mytes to read
     * @param repeated Use a repeated START/START/STOP transaction if true, or independent START/STOP/START/STOP transactions if fasle. Default: true
     *
     * @return DEVICE_OK or DEVICE_I2C_ERROR if the the read request failed.
     */
int ZI2C::readRegister(uint16_t address, uint8_t reg, uint8_t *data, int length, bool repeated)
{
    // write followed by a read...
    int ret = write(address, &reg, 1, repeated);

    if (ret)
        return ret;

    ret = read(address, data, length, false);
    return ret;
}
