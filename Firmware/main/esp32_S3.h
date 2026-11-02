#ifndef ESP32_S3_H
#define ESP32_S3_H

#include "stdio.h"
#include "driver/gpio.h"  //ESP32 GPIO
#include "driver/adc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "soc/soc.h"
#include "esp_intr_alloc.h"
#include "soc/interrupts.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "math.h"
#include "fcntl.h"
#include "driver/usb_serial_jtag.h"
#include "driver/twai.h"


// GENERAL GPIO //
#define MOTOR_ID 1

#define SERIAL_RATE 100   

#define USB_DP 20
#define USB_DN 19

#define BOOT 0

#define CAN_RX ((gpio_num_t)44)
#define CAN_TX ((gpio_num_t)43)
#define BAUDRATE 115200

// DRV8302 GPIO //
#define A_SENSE ADC1_CHANNEL_0  // GPIO 1
#define B_SENSE ADC1_CHANNEL_1  // GPIO 2
#define C_SENSE ADC1_CHANNEL_2  // GPIO 3

#define INL_A ((gpio_num_t)15)
#define INH_A ((gpio_num_t)16)
#define INL_B ((gpio_num_t)6)
#define INH_B ((gpio_num_t)7)
#define INL_C ((gpio_num_t)4)
#define INH_C ((gpio_num_t)5)

#define DC_CAL ((gpio_num_t)39)
#define MPWM ((gpio_num_t)40)
#define MOC ((gpio_num_t)41)
#define NFAULT ((gpio_num_t)42)
#define NOCTW ((gpio_num_t)38)

#define GAIN ((gpio_num_t)37)
#define SO1 ADC1_CHANNEL_7  // GPIO 8
#define SO2 ADC1_CHANNEL_8  // GPIO 9
#define SO1_GPIO ((gpio_num_t)8)
#define SO2_GPIO ((gpio_num_t)9)

#define EN_DRV ((gpio_num_t)14)

// SENSORS //

#define HALL_A ((gpio_num_t)10)
#define HALL_B ((gpio_num_t)11)
#define HALL_C ((gpio_num_t)12)

#define BMI_SDA ((gpio_num_t)47) // 3 Wire SPI
#define BMI_SCK ((gpio_num_t)35)
#define BMI_CS ((gpio_num_t)36)

#define MAG_DO ((gpio_num_t)48)   // 3 Wire SPI - One way (SSI)
#define MAG_CSN ((gpio_num_t)18)
#define MAG_CLK ((gpio_num_t)21)

#define NTC ADC2_CHANNEL_2  // GPIO 13
#define PVDD_SENSE ADC2_CHANNEL_6  // GPIO 17

enum SystemState {
    SYSTEM_INIT,
    SYSTEM_CALIBRATION,
    SYSTEM_RUNNING,
};

enum FeedbackSys{
    MAG,
    HALL,
};

enum DRV_MotionState {
    MOTION_IDLE,
    MOTION_STARTUP,
    MOTION_RAMPING,
    MOTION_SPINNING,
    MOTION_RAMPING_DOWN
};

enum StatusFlags {
    STATUS_OVERCURRENT = 1 << 0,
    STATUS_OVERTEMP    = 1 << 1,
    STATUS_UNDERVOLT   = 1 << 2,
    STATUS_FAULT       = 1 << 3,
    STATUS_OK          = 1 << 4
};

enum ControlMode {
    MODE_OPENLOOP_BEMF,     // trapezoidal open loop
    MODE_CLOSED_FOC,
    MODE_CLOSED_SINUSOIDAL
};

enum ClosedMode {
    MODE_VEL,
    MODE_POS,
    MODE_TORQUE,
    MODE_IDLE
};

#pragma pack(push, 1)
typedef struct {
    uint8_t ID;
    float pvdd;
    float i1;
    float i2;
    float i3;
    float temp;
    float angle;
    float red_angle;
    float rpm;
    float red_rpm;

    uint16_t A;
    uint16_t B;
    uint16_t C;

    uint8_t nfault;
    uint8_t oc;
/*
    float accx;
    float accy;
    float accz;
    float gyrx;
    float gyry;
    float gyrz;
    uint8_t ffall;*/
} TelemetryPacket;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint8_t ID;
    uint8_t status;
    uint8_t mode;
    float value_fbk;
} CANmotorPacket;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint8_t ID;
    uint8_t mode;
    float value_cmd;
} CANMasterPacket;
#pragma pack(pop)

typedef struct {
    float angle;
    float reducer_angle;
    uint8_t status;
} angle_data;
    
//Timer 1MHz
/*/
#define TIMER_DIVIDER 40          // 40 MHz / 80 = 0.5 MHz timer
#define TIMER_INTERVAL_US 25      // 25 µs → 40 kHz

#define FOC_DIVIDER        1    //40khz (max 200 khz)
#define FBK_DIVIDER        40    //10khz
#define BMI_DIVIDER        40   //1khz
#define DRV_DIVIDER        80   //500Hz  
#define TELEMETRY_DIVIDER  2000  // 20hz*/


//Prior
#define TIMER_DIVIDER 80          // 40 MHz / 80 = 0.5 MHz timer
#define TIMER_INTERVAL_US 50      // 50 µs → 20 kHz

#define CONTROL_DIVIDER        1    //20khz (max 200 khz)
#define BMI_DIVIDER        20   //1khz
#define CAN_DIVIDER        5   //4kHz 
#define TELEMETRY_DIVIDER  500  // 40hz

#endif





