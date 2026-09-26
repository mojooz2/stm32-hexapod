#include "servo.h"

#define PCA9685_1_ADDR       (0x40 << 1)
#define PCA9685_2_ADDR       (0x41 << 1)

#define PCA9685_MODE1        0x00
#define PCA9685_PRESCALE     0xFE

#define NUM_SERVOS           18
#define SERVOS_PER_PCA       9

extern I2C_HandleTypeDef hi2c1;


/*
 * Write one register to a specified PCA9685.
 */
static void PCA9685_WriteRegister(uint16_t address,
                                  uint8_t reg,
                                  uint8_t value)
{
    HAL_I2C_Mem_Write(&hi2c1,
                      address,
                      reg,
                      I2C_MEMADD_SIZE_8BIT,
                      &value,
                      1,
                      100);
}


/*
 * Read one register from a specified PCA9685.
 */
static uint8_t PCA9685_ReadRegister(uint16_t address,
                                    uint8_t reg)
{
    uint8_t value = 0;

    HAL_I2C_Mem_Read(&hi2c1,
                     address,
                     reg,
                     I2C_MEMADD_SIZE_8BIT,
                     &value,
                     1,
                     100);

    return value;
}


/*
 * Set PWM frequency for a specified PCA9685.
 */
static void PCA9685_SetPWMFreq(uint16_t address,
                               uint8_t prescale)
{
    uint8_t oldmode;

    oldmode = PCA9685_ReadRegister(address, PCA9685_MODE1);

    /* Enter sleep mode so PRESCALE can be changed. */
    PCA9685_WriteRegister(address,
                          PCA9685_MODE1,
                          (oldmode & 0x7F) | 0x10);

    PCA9685_WriteRegister(address,
                          PCA9685_PRESCALE,
                          prescale);

    /* Restore previous mode. */
    PCA9685_WriteRegister(address,
                          PCA9685_MODE1,
                          oldmode);

    HAL_Delay(5);

    /* Restart + auto-increment. */
    PCA9685_WriteRegister(address,
                          PCA9685_MODE1,
                          (oldmode & 0xEF) | 0xA0);
}


/*
 * Set PWM output for one physical PCA9685 channel.
 */
static void PCA9685_SetPWM(uint16_t address,
                           uint8_t channel,
                           uint16_t on,
                           uint16_t off)
{
    uint8_t reg = 0x06 + (4 * channel);
    uint8_t data[4];

    data[0] = on & 0xFF;
    data[1] = (on >> 8) & 0x0F;
    data[2] = off & 0xFF;
    data[3] = (off >> 8) & 0x0F;

    HAL_I2C_Mem_Write(&hi2c1,
                      address,
                      reg,
                      I2C_MEMADD_SIZE_8BIT,
                      data,
                      4,
                      100);
}


/*
 * Initialise both PCA9685 boards.
 */
void Servo_Init(void)
{
    PCA9685_SetPWMFreq(PCA9685_1_ADDR, 121);
    PCA9685_SetPWMFreq(PCA9685_2_ADDR, 121);
}


/*
 * Set the angle of logical Servo 0-17.
 */
void Servo_SetAngle(uint8_t servo,
                    float angle)
{
    uint16_t address;
    uint8_t channel;

    uint16_t pulse_neg90;
    uint16_t pulse_0;
    uint16_t pulse_pos90;
    uint16_t pulse;

    /*
     * Only logical servos 0-17 exist.
     */
    if (servo >= NUM_SERVOS)
    {
        return;
    }

    /*
     * Logical servo -> PCA9685 + physical channel.
     *
     * Servo 0-8  -> PCA #1, channels 0-8
     * Servo 9-17 -> PCA #2, channels 0-8
     */
    if (servo < SERVOS_PER_PCA)
    {
        address = PCA9685_1_ADDR;
        channel = servo;
    }
    else
    {
        address = PCA9685_2_ADDR;
        channel = servo - SERVOS_PER_PCA;
    }

    /*
     * Select calibration.
     */
    switch (servo)
    {
        case 0:
            pulse_neg90 = SERVO0_PULSE_NEG90;
            pulse_0 = SERVO0_PULSE_0;
            pulse_pos90 = SERVO0_PULSE_POS90;
            break;

        case 1:
            pulse_neg90 = SERVO1_PULSE_NEG90;
            pulse_0 = SERVO1_PULSE_0;
            pulse_pos90 = SERVO1_PULSE_POS90;
            break;

        case 2:
            pulse_neg90 = SERVO2_PULSE_NEG90;
            pulse_0 = SERVO2_PULSE_0;
            pulse_pos90 = SERVO2_PULSE_POS90;
            break;

        case 3:
            pulse_neg90 = SERVO3_PULSE_NEG90;
            pulse_0 = SERVO3_PULSE_0;
            pulse_pos90 = SERVO3_PULSE_POS90;
            break;

        case 4:
            pulse_neg90 = SERVO4_PULSE_NEG90;
            pulse_0 = SERVO4_PULSE_0;
            pulse_pos90 = SERVO4_PULSE_POS90;
            break;

        case 5:
            pulse_neg90 = SERVO5_PULSE_NEG90;
            pulse_0 = SERVO5_PULSE_0;
            pulse_pos90 = SERVO5_PULSE_POS90;
            break;

        case 6:
            pulse_neg90 = SERVO6_PULSE_NEG90;
            pulse_0 = SERVO6_PULSE_0;
            pulse_pos90 = SERVO6_PULSE_POS90;
            break;

        case 7:
            pulse_neg90 = SERVO7_PULSE_NEG90;
            pulse_0 = SERVO7_PULSE_0;
            pulse_pos90 = SERVO7_PULSE_POS90;
            break;

        case 8:
            pulse_neg90 = SERVO8_PULSE_NEG90;
            pulse_0 = SERVO8_PULSE_0;
            pulse_pos90 = SERVO8_PULSE_POS90;
            break;

        case 9:
            pulse_neg90 = SERVO9_PULSE_NEG90;
            pulse_0 = SERVO9_PULSE_0;
            pulse_pos90 = SERVO9_PULSE_POS90;
            break;

        case 10:
            pulse_neg90 = SERVO10_PULSE_NEG90;
            pulse_0 = SERVO10_PULSE_0;
            pulse_pos90 = SERVO10_PULSE_POS90;
            break;

        case 11:
            pulse_neg90 = SERVO11_PULSE_NEG90;
            pulse_0 = SERVO11_PULSE_0;
            pulse_pos90 = SERVO11_PULSE_POS90;
            break;

        case 12:
            pulse_neg90 = SERVO12_PULSE_NEG90;
            pulse_0 = SERVO12_PULSE_0;
            pulse_pos90 = SERVO12_PULSE_POS90;
            break;

        case 13:
            pulse_neg90 = SERVO13_PULSE_NEG90;
            pulse_0 = SERVO13_PULSE_0;
            pulse_pos90 = SERVO13_PULSE_POS90;
            break;

        case 14:
            pulse_neg90 = SERVO14_PULSE_NEG90;
            pulse_0 = SERVO14_PULSE_0;
            pulse_pos90 = SERVO14_PULSE_POS90;
            break;

        case 15:
            pulse_neg90 = SERVO15_PULSE_NEG90;
            pulse_0 = SERVO15_PULSE_0;
            pulse_pos90 = SERVO15_PULSE_POS90;
            break;

        case 16:
            pulse_neg90 = SERVO16_PULSE_NEG90;
            pulse_0 = SERVO16_PULSE_0;
            pulse_pos90 = SERVO16_PULSE_POS90;
            break;

        case 17:
            pulse_neg90 = SERVO17_PULSE_NEG90;
            pulse_0 = SERVO17_PULSE_0;
            pulse_pos90 = SERVO17_PULSE_POS90;
            break;

        default:
            // Optional: handle unexpected servo indices
            break;
    }

    /*
     * Limit requested angle to the calibrated range.
     */
    if (angle < -90.0f)
    {
        angle = -90.0f;
    }

    if (angle > 90.0f)
    {
        angle = 90.0f;
    }

    /*
     * Piecewise-linear conversion:
     *
     * -90 deg -> 0 deg
     */
    if (angle <= 0.0f)
    {
        pulse = pulse_neg90 +
                (uint16_t)(((angle + 90.0f) / 90.0f) *
                (pulse_0 - pulse_neg90));
    }

    /*
     * 0 deg -> +90 deg
     */
    else
    {
        pulse = pulse_0 +
                (uint16_t)((angle / 90.0f) *
                (pulse_pos90 - pulse_0));
    }

    PCA9685_SetPWM(address,
                   channel,
                   0,
                   pulse);
}
