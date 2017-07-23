#include "config.h"

void set_pins(void)
{
    // BUTTON pins
    palSetPadMode(GPIOE, 2, PAL_MODE_INPUT_PULLDOWN);
    palSetPadMode(GPIOE, 3, PAL_MODE_INPUT_PULLDOWN);
    palSetPadMode(GPIOE, 4, PAL_MODE_INPUT_PULLDOWN);
    palSetPadMode(GPIOE, 5, PAL_MODE_INPUT_PULLDOWN);
    palSetPadMode(GPIOE, 6, PAL_MODE_INPUT_PULLDOWN);

    // LED pins
    palSetPadMode(GPIOE, 0, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOE, 1, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOB, 8, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOB, 9, PAL_MODE_OUTPUT_PUSHPULL);

    palClearPad(GPIOE, 0);
    palClearPad(GPIOE, 1);
    palClearPad(GPIOB, 8);
    palClearPad(GPIOB, 9);

    // UART pins
    palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7));  // USART_1
    palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));

    palSetPadMode(GPIOD, 5, PAL_MODE_ALTERNATE(7)); // USART_2
    palSetPadMode(GPIOD, 6, PAL_MODE_ALTERNATE(7));

    palSetPadMode(GPIOD, 8, PAL_MODE_INPUT_PULLDOWN); //                        // PAL_MODE_ALTERNATE(7)); // USART_3
    palSetPadMode(GPIOD, 9, PAL_MODE_OUTPUT_PUSHPULL); // DRIBLER_CONTROLL      // PAL_MODE_ALTERNATE(7));

    palSetPadMode(GPIOC, 10, PAL_MODE_ALTERNATE(8)); // UART_4
    palSetPadMode(GPIOC, 11, PAL_MODE_ALTERNATE(8));

    palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(8)); // USART_6
    palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(8));

	// KICKER pins
    palSetPadMode(GPIOC, 12, PAL_MODE_OUTPUT_PUSHPULL); // KICK
    palSetPadMode(GPIOD, 2, PAL_MODE_INPUT_PULLDOWN); // READY

    // BALL_DETECTON pin
    palSetPadMode(GPIOD, 15, PAL_MODE_INPUT_PULLDOWN); //BALL_DETECTON

    // ADC pins
    palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_PULLDOWN);
    palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG);

    // PWM pins
    palSetPadMode(GPIOD, 12, PAL_MODE_OUTPUT_PUSHPULL);
    palSetPadMode(GPIOD, 13, PAL_MODE_ALTERNATE(2));
    palSetPadMode(GPIOD, 14, PAL_MODE_ALTERNATE(2));

    palSetPadMode(GPIOE, 9, PAL_MODE_INPUT_PULLDOWN);

    palSetPadMode(GPIOE, 11, PAL_MODE_ALTERNATE(1));
    palSetPadMode(GPIOE, 13, PAL_MODE_ALTERNATE(1));
    palSetPadMode(GPIOE, 14, PAL_MODE_ALTERNATE(1));

    // I2C
    palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(4)); // COMPASS
    palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(4));

    palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(4)); // I2C3
    palSetPadMode(GPIOC, 9, PAL_MODE_ALTERNATE(4));
}

static const SerialConfig usart_1_cfg = {
    115200,
    0,
    0,
    0
};

static const SerialConfig usart_2_cfg = {
    115200,
    0,
    0,
    0
};

static const SerialConfig usart_3_cfg = {
    9600,
    0,
    0,
    0
};

static const SerialConfig usart_4_cfg = {
    57600,
    0,
    0,
    0
};

static const SerialConfig usart_6_cfg = {
    115200,
    0,
    0,
    0
};


static const I2CConfig compass_i2c_cfg = {
    OPMODE_I2C,
    10000,
    FAST_DUTY_CYCLE_16_9
};

void init_drivers(void) {
    sdInit();

    sdStart(&SD1, &usart_1_cfg);
    sdStart(&SD2, &usart_2_cfg);
    sdStart(&SD3, &usart_3_cfg);
    sdStart(&SD4, &usart_4_cfg);
    sdStart(&SD6, &usart_6_cfg);

    i2cInit();

    i2cObjectInit(&I2CD1);  // compass i2c (I2CD1)
    i2cStart(&I2CD1,&compass_i2c_cfg);

    adcStart(&ADCD1, NULL);
    pwmInit();

}

void board_init(void) {
    set_pins();
    init_drivers();
}
