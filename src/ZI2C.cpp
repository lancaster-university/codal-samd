#include "ZI2C.h"
#include "codal_target_hal.h"
#include "hal_gpio.h"
#include "CodalDmesg.h"

using namespace codal;

#define MAX_I2C_RETRIES     2

void ZI2C::reset()
{
    i2c_m_sync_disable(&this->i2c);
    i2c_m_sync_deinit(&this->i2c);

    DMESG("R RECOVERY!");
    gpio_set_pin_function(this->sda.name, GPIO_PIN_FUNCTION_OFF);
    gpio_set_pin_function(this->scl.name, GPIO_PIN_FUNCTION_OFF);

    gpio_set_pin_direction(this->sda.name, GPIO_DIRECTION_OUT);
    gpio_set_pin_direction(this->scl.name, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(this->sda.name,false);
    gpio_set_pin_level(this->scl.name,false);

    target_wait_us(10);

    gpio_set_pin_level(this->sda.name,true);
    gpio_set_pin_level(this->scl.name,true);

    for (int i = 0; i < 8; i++)
    {
        gpio_set_pin_level(this->scl.name,true);
        target_wait_us(3);
        gpio_set_pin_level(this->scl.name,false);
        target_wait_us(3);
    }

    gpio_set_pin_function(this->scl.name,GPIO_DIRECTION_IN);
    gpio_set_pin_function(this->sda.name,GPIO_DIRECTION_IN);
    gpio_set_pin_pull_mode(this->scl.name, GPIO_PULL_OFF);
    gpio_set_pin_pull_mode(this->sda.name, GPIO_PULL_OFF);
    scl._setMux(sclMux);
    sda._setMux(sdaMux);

    memset(&i2c, 0, sizeof(i2c_m_sync_desc));
    int ret = 0;
    ret = i2c_m_sync_init(&i2c, this->instance);
    // i2c->STATUS.bit.BUSSTATE;
    DMESG("INIT ret: %d",ret);
    ret = i2c_m_sync_set_baudrate(&i2c, 0, 100); // set i2c freq to 100khz
    DMESG("baud ret: %d",ret);

    ret = i2c_m_sync_enable(&i2c);
    DMESG("en ret: %d",ret);

    this->instance->I2CM.STATUS.bit.BUSSTATE = 1;
    while(this->instance->I2CM.SYNCBUSY.bit.SYSOP);
}

/**
 * Constructor.
 */
ZI2C::ZI2C(ZPin &sda, ZPin &scl) : codal::I2C(sda,scl), sda(sda), scl(scl)
{
    const mcu_pin_obj_t* sda_pin = samd_peripherals_get_pin(sda.name);
    const mcu_pin_obj_t* scl_pin = samd_peripherals_get_pin(scl.name);

    DMESG("SDA %d, SCL %d",sda.name,scl.name);

    int sercomIdx = -1;
    int sda_fun = -1;
    int scl_fun = -1;

    if (sda_pin->sercom[0].index != 0x3f && sda_pin->sercom[0].pad == 0)
    {
        sda_fun = MUX_C; // c
        sercomIdx = sda_pin->sercom[0].index;
    } else if (sda_pin->sercom[1].index != 0x3f && sda_pin->sercom[1].pad == 0)
    {
        sda_fun = MUX_D; // d
        sercomIdx = sda_pin->sercom[1].index;
    } else
        target_panic(DEVICE_HARDWARE_CONFIGURATION_ERROR);

    if (scl_pin->sercom[0].index != 0x3f && scl_pin->sercom[0].index == sercomIdx && scl_pin->sercom[0].pad == 1)
        scl_fun = MUX_C; // c
    else if (scl_pin->sercom[1].index != 0x3f && scl_pin->sercom[1].index == sercomIdx && scl_pin->sercom[1].pad == 1)
        scl_fun = MUX_D; // d
    else
        target_panic(DEVICE_HARDWARE_CONFIGURATION_ERROR);

    gpio_set_pin_direction(sda_pin->number, GPIO_DIRECTION_OUT);
    gpio_set_pin_direction(scl_pin->number, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(sda_pin->number,true);
    gpio_set_pin_level(scl_pin->number,true);

    gpio_set_pin_function(sda_pin->number, GPIO_PIN_FUNCTION_OFF);
    gpio_set_pin_function(scl_pin->number, GPIO_PIN_FUNCTION_OFF);
    gpio_set_pin_direction(sda_pin->number, GPIO_DIRECTION_IN);
    gpio_set_pin_direction(scl_pin->number, GPIO_DIRECTION_IN);

    gpio_set_pin_pull_mode(sda_pin->number, GPIO_PULL_DOWN);
    gpio_set_pin_pull_mode(scl_pin->number, GPIO_PULL_DOWN);

    target_wait_us(10);

    gpio_set_pin_pull_mode(sda_pin->number, GPIO_PULL_OFF);
    gpio_set_pin_pull_mode(scl_pin->number, GPIO_PULL_OFF);

    // target_wait_us(3);

    // if (!gpio_get_pin_level(sda_pin->number) || !gpio_get_pin_level(scl_pin->number)) {

    //     while(1);
    //     // reset_pin_number(sda_pin->number);
    //     // reset_pin_number(scl_pin->number);
    //     // mp_raise_RuntimeError(translate("SDA or SCL needs a pull up"));
    // }

    DMESG("SDA pad %d, idx %d, fn: %d", sercomIdx, sda_fun);
    DMESG("SCL pad %d, idx %d, fn: %d", sercomIdx, scl_fun);

    sclMux = scl_fun;
    sdaMux = sda_fun;

    sda._setMux(sda_fun);
    scl._setMux(scl_fun);

    this->instance = sercom_insts[sercomIdx];
    samd_peripherals_sercom_clock_init(this->instance, sercomIdx);

    memset(&i2c, 0, sizeof(i2c_m_sync_desc));
    int ret = 0;
    ret = i2c_m_sync_init(&i2c, this->instance);
    // i2c->STATUS.bit.BUSSTATE;
    DMESG("INIT ret: %d",ret);
    ret = i2c_m_sync_set_baudrate(&i2c, 0, 100); // set i2c freq to 100khz
    DMESG("baud ret: %d",ret);

    ret = i2c_m_sync_enable(&i2c);
    DMESG("en ret: %d",ret);

    this->instance->I2CM.STATUS.bit.BUSSTATE = 1;
    while(this->instance->I2CM.SYNCBUSY.bit.SYSOP);
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
    int busState = ((Sercom*)&i2c.device.hw)->I2CM.STATUS.bit.BUSSTATE;

    // DMESG("BS %d",busState);

    if (busState == 0)
    {
        DMESG("FORCING STATE");
        ((Sercom*)&i2c.device.hw)->I2CM.STATUS.bit.BUSSTATE = 1;
        while(((Sercom*)&i2c.device.hw)->I2CM.SYNCBUSY.bit.SYSOP);
    }
    address = address >> 1;
    // DMESG("W A: %d L: %d", address, len);
    struct _i2c_m_msg msg;

    int ret = I2C_OK;

    for (int i = 0; i < MAX_I2C_RETRIES; i++)
    {
        msg.addr = address;
        msg.len = len;
        msg.flags  = repeated ? 0 : I2C_M_STOP;
        msg.buffer = (uint8_t *) data;
        ret = _i2c_m_sync_transfer(&i2c.device, &msg);

        if (ret == I2C_OK)
        {
            // DMESG("OK");
            return DEVICE_OK;
        }
    }

    if (ret == -5 || ret == -4)
    {
        DMESG("W RECOVERY!");
        this->reset();
    }

    DMESG("W HAL ERR %d",ret);
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
    int busState = ((Sercom*)&i2c.device.hw)->I2CM.STATUS.bit.BUSSTATE;

    DMESG("BS %d",busState);

    if (busState == 0)
    {
        DMESG("FORCING STATE");
        ((Sercom*)&i2c.device.hw)->I2CM.STATUS.bit.BUSSTATE = 1;
        while(((Sercom*)&i2c.device.hw)->I2CM.SYNCBUSY.bit.SYSOP);
    }

    address = address >> 1;
    // DMESG("R A: %d, L: %d", address, len);
    struct _i2c_m_msg msg;

    int ret = I2C_OK;

    for (int i = 0; i < MAX_I2C_RETRIES; i++)
    {
        msg.addr   = address;
        msg.len    = len;
        msg.flags  = (repeated ? 0 : I2C_M_STOP) | I2C_M_RD;
        msg.buffer = data;
        ret = _i2c_m_sync_transfer(&i2c.device, &msg);

        if (ret == I2C_OK)
        {
            // DMESG("OK");
            return DEVICE_OK;
        }
    }

    if (ret == -5 || ret == -4)
    {
        DMESG("R RECOVERY!");
        this->reset();
    }

    DMESG("R HAL ERR %d",ret);
    return DEVICE_I2C_ERROR;
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
