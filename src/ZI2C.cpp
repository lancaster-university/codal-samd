#include "ZI2C.h"
#include "codal_target_hal.h"
#include "hal_gpio.h"
#include "CodalDmesg.h"
#include "CodalConfig.h"

using namespace codal;

#define MAX_I2C_RETRIES     2

void SERCOM1_IRQHandler()
{
    CODAL_ASSERT(0);
}

inline int spin(Sercom* inst)
{
    uint16_t timeout = 0xFFFF;

    do
    {
        // send the stop bit if timeout is reached.
        if (timeout-- == 0)
        {
            inst->I2CM.CTRLB.bit.CMD = 0x3;
            return DEVICE_I2C_ERROR;
        }

    // master or slave on bus?!
    }while (!(inst->I2CM.INTFLAG.bit.MB) && !(inst->I2CM.INTFLAG.bit.SB));

    return DEVICE_OK;
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

    target_wait_us(3);

    // if (!gpio_get_pin_level(sda_pin->number) || !gpio_get_pin_level(scl_pin->number)) {

    //     while(1);
    //     // reset_pin_number(sda_pin->number);
    //     // reset_pin_number(scl_pin->number);
    //     // mp_raise_RuntimeError(translate("SDA or SCL needs a pull up"));
    // }

    DMESG("SDA idx %d, fn: %d", sercomIdx, sda_fun);
    DMESG("SCL idx %d, fn: %d", sercomIdx, scl_fun);

    sclMux = scl_fun;
    sdaMux = sda_fun;

    gpio_set_pin_function(sda_pin->number, sda_fun);
    gpio_set_pin_function(scl_pin->number, scl_fun);

    this->instance = sercom_insts[sercomIdx];
    samd_peripherals_sercom_clock_init(this->instance, sercomIdx);

    DMESG("INST: %p %d",this->instance, sercomIdx);

    NVIC_DisableIRQ(SERCOM1_IRQn);
    NVIC_DisableIRQ(SERCOM1_IRQn);
    NVIC_DisableIRQ(SERCOM1_IRQn);
    NVIC_DisableIRQ(SERCOM1_IRQn);

    disable();

    this->instance->I2CM.CTRLA.bit.SWRST = 1;
    while(this->instance->I2CM.SYNCBUSY.bit.SWRST);

    this->instance->I2CM.CTRLA.bit.MODE = 0x5; // master mode
    this->instance->I2CM.CTRLA.bit.SPEED = 0; // 100 - 400 khz
    // // this->instance->I2CM.CTRLA.bit.MEXTTOEN = 1; // timeout if scl is held lo
    this->instance->I2CM.CTRLA.bit.PINOUT = 0; // two wire mode


    setFrequency(100000); // freq in hz

    // this->instance->I2CM.STATUS.bit.RXNACK; // if 0 ACK, if 1 NACK.
    // this->instance->I2CM.STATUS.bit.BUSERR; // illegal bus condition

    // // assume we're the only master
    this->instance->I2CM.STATUS.bit.BUSSTATE = 1;
    while(this->instance->I2CM.SYNCBUSY.bit.SYSOP);

    enable();

    target_wait_us(100);
}

int ZI2C::disable()
{
    this->instance->I2CM.CTRLA.bit.ENABLE = 0;
    while(this->instance->I2CM.SYNCBUSY.bit.ENABLE);
    return DEVICE_OK;
}

int ZI2C::enable()
{
    this->instance->I2CM.CTRLA.bit.ENABLE = 1;
    while(this->instance->I2CM.SYNCBUSY.bit.ENABLE);
    return DEVICE_OK;
}

/** Set the frequency of the I2C interface
 *
 * @param frequency The bus frequency in hertz
 */
int ZI2C::setFrequency(uint32_t frequency)
{
    uint32_t clk_rate = CODAL_CPU_MHZ * 1000; // (clk speed khz)
    frequency /= 1000;// convert hertz to khz

    // uint32_t baud = (uint32_t)(clk_rate / (2 * frequency) - 5 - (clk_rate * 90 / 2));

    this->instance->I2CM.BAUD.bit.BAUD = 52; // SCL timing
    this->instance->I2CM.BAUD.bit.BAUDLOW = 0; // sda timing copied from baud

    return DEVICE_OK;
}

/**
* Issues a standard, I2C CMD write to the I2C bus.
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
    address &= ~0x01; // this is a write, unset lowest bit.

    uint16_t status = this->instance->I2CM.STATUS.reg;
    uint32_t iflag = this->instance->I2CM.INTFLAG.reg;

    DMESG("STAT B4 %d %d", status, iflag);

    if (iflag & 0x1)
    {
        DMESG("SET IFLAG");
        this->instance->I2CM.INTFLAG.reg = 0;
    }

    int ret = DEVICE_OK;

    this->instance->I2CM.ADDR.bit.ADDR = address;

    if ((ret = spin(this->instance)) != DEVICE_OK)
    {
        DMESG("ERR w ADDR");
        return ret;
    }

    for (int i = 0; i < len; i++)
    {
        uint16_t status = this->instance->I2CM.STATUS.reg;

        // arb lost buserr
        if (status & 0x7)
        {
            DMESG("ERR %d", status);
            return DEVICE_I2C_ERROR;
        }

        this->instance->I2CM.DATA.bit.DATA = data[i];

        // if ((ret = spin(this->instance)) != DEVICE_OK)
        // {
        //     DMESG("ERR WRITE");
        //     return ret;
        // }
    }

    if (!repeated)
        this->instance->I2CM.CTRLB.bit.CMD = 0x3;

    return ret;
}

/**
 * Issues a standard, 2 byte I2C CMD read to the I2C bus.
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
    DMESG("BS %d", this->instance->I2CM.STATUS.bit.BUSSTATE);
    address |= 0x01; // this is a read, set lowest bit.

    int ret = DEVICE_OK;

    this->instance->I2CM.ADDR.bit.ADDR = address;

    if ((ret = spin(this->instance)) != DEVICE_OK)
    {
        DMESG("ERR r ADDR");
        return ret;
    }

    this->instance->I2CM.CTRLB.bit.ACKACT = 0;

    for (int i = 0; i < len; i++)
    {
        data[i] = this->instance->I2CM.DATA.bit.DATA;

        // if ((ret = spin(this->instance)) != DEVICE_OK)
        // {
        //     DMESG("ERR READ");
        //     return ret;
        // }
    }

    if (!repeated)
    {
        // this->instance->I2CM.CTRLB.bit.ACKACT = 1;
        this->instance->I2CM.CTRLB.bit.CMD = 0x3;
    }

    return ret;
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
    DMESG("ADDR %d reg %d", address, reg);
    // write followed by a read...
    int ret = write(address, &reg, 1, repeated);

    if (ret != DEVICE_OK)
        return ret;

    ret = read(address, data, length, false);
    return ret;
}
