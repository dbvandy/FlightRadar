#pragma once

// CrowPanel Advanced 10.1" ESP32-P4 (V1.2): EK79007 1024x600 MIPI-DSI + GT911.

#define PIN_LED             (48)
#define LED_ON              (1)
#define LED_OFF             (0)

#define Touch_GPIO_RST      (40)
#define Touch_GPIO_INT      (42)

#define I2C_GPIO_SCL        (46)
#define I2C_GPIO_SDA        (45)

#define LCD_GPIO_BLIGHT     (31)
#define BLIGHT_PWM_Hz       (30000)
#define BLIGHT_ON_LEVEL     (1)

#define LCD_GPIO_RST        (41)

#define V_size              (600)
#define H_size              (1024)
#define LCD_CLK_MHZ         (51)
#define LCD_HPW             (70)
#define LCD_HBP             (160)
#define LCD_HFP             (160)
#define LCD_VPW             (10)
#define LCD_VBP             (23)
#define LCD_VFP             (21)
