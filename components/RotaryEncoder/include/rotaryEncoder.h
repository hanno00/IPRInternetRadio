#ifndef RE_H
#define RE_H

 #ifdef __cplusplus
extern "C" {
#endif
// Error library
#include "esp_err.h"
// I2C driver
#include "driver/i2c.h"
// FreeRTOS (for delay)
#include "freertos/task.h"

#define RE_I2C_ADDR 0x3F
#define PORT        0x00

typedef enum {
    RE_ERR_OK      = 0x00,
    RE_ERR_CONFIG  = 0x01,
    RE_ERR_INSTALL = 0x02,
    RE_ERR_FAIL    = 0x03
}  RE_err_t;

#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

#ifndef WRITE_BIT
#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#endif

#ifndef READ_BIT
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#endif

// bit to enable checking for ACK
#ifndef ACK_CHECK_EN
#define ACK_CHECK_EN   0x1
#endif

typedef enum{
    INT_ENABLE = 0x04,             //Set the bit to 1 to enable the interrupt for the button or encoder.
    LED_BRIGHTNESS_RED = 0x0D,     //Value between 0 and 255 representing the brightness of the red LED
    LED_BRIGHTNESS_GRN = 0x0E,     //Value between 0 and 255 representing the brightness of the green LED
    LED_BRIGHTNESS_BLU = 0x0F,     //Value between 0 and 255 representing the brightness of the blue LED
    STAT = 0x01,                   //Status of the Qwiic Twist. 3:0 = buttonPressed(3),
                                   //buttonClicked(2), buttonInterrupt(1), encoderInterrupt(0). 
                                   //Events must be cleared by user.
    FW_VERSION_MSB = 0x02,         //The current firmware version
    FW_VERSION_LSB = 0x03,
    RE_COUNT_LSB = 0x05,
    RE_COUNT_MSB = 0x06,
    RE_DIFF_LSB = 0x07,
    RE_DIFF_MSB = 0x08,
    RE_TSLM_LSB = 0x09,
    RE_TSLM_MSB = 0x0A,
    RE_TSLB_LSB = 0x0B,
    RE_TSLB_MSB = 0x0C,
    LED_CON_RED_LSB = 0x10,
    LED_CON_RED_MSB = 0x11,
    LED_CON_GRN_LSB = 0x12,
    LED_CON_GRN_MSB = 0x13,
    LED_CON_BLU_LSB = 0x14,
    LED_CON_BLU_MSB = 0x15,
    TRN_INT_TO_LSB = 0x16,
    TRN_INT_TO_MSB = 0x17,
}RE_reg_t;

// init for the i2c connection
RE_err_t RotaryEncoder_init();
// check if RE is Connected
bool RotaryEncoder_isConnected();
//Status of the Qwiic Twist. 3:0 = buttonPressed(3), buttonClicked(2), buttonInterrupt(1), encoderInterrupt(0). 
RE_err_t RotaryEncoder_getStatus(uint8_t *data);
//Set the bit to 1 to enable the interrupt for the button or encoder. buttonInterruptEnable(1), encoderInterruptEnable(0). Default is 0x03
RE_err_t RotaryEncoder_interruptEnable(bool v);
//Value between 0 and 255 representing the brightness of the red LED. Default is 255
RE_err_t RotaryEncoder_setColorRed(uint8_t v);
//Value between 0 and 255 representing the brightness of the green LED. Default is 255
RE_err_t RotaryEncoder_setColorGrn(uint8_t v);
//Value between 0 and 255 representing the brightness of the blue LED. Default is 255
RE_err_t RotaryEncoder_setColorBlu(uint8_t v);
//Value between 255 and -255 indicating the amount to change the red LED brightness with each tick movement of the encoder. Default is 0
RE_err_t RotaryEncoder_conectColorRed(uint16_t v);
//Value between 255 and -255 indicating the amount to change the green LED brightness with each tick movement of the encoder. Default is 0
RE_err_t RotaryEncoder_conectColorGrn(uint16_t v);
//Value between 255 and -255 indicating the amount to change the blue LED brightness with each tick movement of the encoder. Default is 0
RE_err_t RotaryEncoder_conectColorBlu(uint16_t v);
//Value between 32767 and -32768 indicating the number of ticks the user has twisted the knob. 24 ticks per rotation.
RE_err_t RotaryEncoder_getCount(uint16_t *data);
//Value between 32767 and -32768 indicating the number of ticks the user has twisted the knob since last movement. 
RE_err_t RotaryEncoder_getDiff(uint16_t *data);
//Value between 0 and 65535 indicating the number of milliseconds since last movement.
RE_err_t RotaryEncoder_getTSLM(uint16_t *data);
//Value between 0 and 65535 indicating the number of milliseconds since last press/release event.
RE_err_t RotaryEncoder_getTSLB(uint16_t *data);



#ifdef __cplusplus
}
#endif

#endif
