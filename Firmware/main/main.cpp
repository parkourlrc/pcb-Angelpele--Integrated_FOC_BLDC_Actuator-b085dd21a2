#include "esp32_S3.h"
#include "DRV.h"
#include "MT6701.h"
#include "bmi.h"
#include "hall.h"
#include "MCP2551.h"

// TIMER //
volatile uint32_t isr_counter = 0;

TaskHandle_t focTaskHandle = NULL;
TaskHandle_t bmiTaskHandle = NULL;
TaskHandle_t drvTaskHandle = NULL;
TaskHandle_t telemetryTaskHandle = NULL;
TaskHandle_t canTaskHandle = NULL;

// MAIN VARIABLE DECLARATION //
DRV drv;
MT6701 mag;
BMI270 bmi270;
CAN CAN_bus;
HALL_SENSOR hall_sensor;
angle_data fbk_pkg;
BMI_data bmi_pkg;
TelemetryPacket pkg;
CANmotorPacket slave_pkg;
CANMasterPacket master_pkg;

SemaphoreHandle_t drv_mutex = NULL;
SemaphoreHandle_t fbk_mutex = NULL;
SemaphoreHandle_t bmi_mutex = NULL;

// MAIN FUNCTION DECLARATION //
int initialize();
int calibration();
void create_tasks_once();
void controlISR(void* arg);
void controlLoop(void* arg);
void fbkLoop(void* arg);
void drvLoop(void* arg);
void telemetryLoop(void* arg);
void bmiLoop(void* arg);
void canLoop(void* arg);

SystemState system_state = SYSTEM_INIT;
DRV_MotionState motion = MOTION_IDLE;
StatusFlags status_flags = STATUS_OK;
FeedbackSys fbk_sys = MAG;  // Default to MAG feedback system
ControlMode control_mode = MODE_CLOSED_FOC;
ClosedMode closed_mode = MODE_POS;

bool debug_mode_enabled = false;
bool included_bmi = false;
bool slave_motor = true;
bool CAN_enabled = true;
uint8_t master_id = 0;

extern "C" void app_main(void)
{
    drv_mutex = xSemaphoreCreateMutex();
    fbk_mutex = xSemaphoreCreateMutex();
    bmi_mutex = xSemaphoreCreateMutex();
    configASSERT(drv_mutex);
    configASSERT(fbk_mutex);
    configASSERT(bmi_mutex);

    esp_task_wdt_add(NULL); // Subscribe main to the task watchdog

    while (true)
    {
        switch (system_state)
        {
            // ---------------- INIT ----------------
            case SYSTEM_INIT:
            {       
                
                gpio_set_pull_mode(CAN_RX, GPIO_PULLUP_ONLY); // GPIO44
                gpio_set_pull_mode(CAN_TX, GPIO_PULLUP_ONLY); // GPIO43

                if (initialize() == 0) {
                    drv.set_override(true);
                    system_state = SYSTEM_CALIBRATION;
                    mag.set_override(fbk_sys == MAG ? false : true);
                    bmi270.set_override(!included_bmi);
                } else {
                    printf("Initialization failed, retrying...\n");
                    status_flags = STATUS_UNDERVOLT;
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                break;
            }
            
            // ---------------- CALIBRATION ----------------
            case SYSTEM_CALIBRATION:
            {
                if (calibration() == 0) {
                    printf("\nRunning....\n");
                    vTaskDelay(pdMS_TO_TICKS(200));
                    create_tasks_once();
                    system_state = SYSTEM_RUNNING;

                } else {
                    printf("\nCalibration failed, retrying....\n");
                    vTaskDelay(pdMS_TO_TICKS(2000));
                }
                break;
            }
            
            // ---------------- MAIN LOOP ----------------
            case SYSTEM_RUNNING:
            {;
                esp_task_wdt_reset();
                if (drv.fault_detection() == DRV_LATCHED) {
                    status_flags = STATUS_FAULT;
                    printf("Critical fault detected. Driver disabled. Waiting for reset....\n");
                    fflush(stdout);
                    system_state = SYSTEM_INIT;
                    gpio_set_level(EN_DRV, 0);
                    
                }
                vTaskDelay(pdMS_TO_TICKS(10));
                break;
            }
        }
    }
}

int initialize() {
    static bool usb_installed = false;
    if (!usb_installed) {
        usb_serial_jtag_driver_config_t usb_cfg = { .tx_buffer_size = 256, .rx_buffer_size = 256 };
        if (usb_serial_jtag_driver_install(&usb_cfg) == ESP_OK) usb_installed = true;
    }
    int status = 0;

    if (drv.init() != DRV_OK) {
        status |= 2;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    hall_sensor.init(CFG_MOTOR_POLE_PAIRS);
    mag.init();
    if (included_bmi) {
        if (bmi270.init() != BMI_OK) {
            status |= 4;
        }
    }

    if (CAN_enabled) {
        if (CAN_bus.init() != 0) {
            status |= 8;
        }
    }
    return status; 
}

int calibration() {
    drv.setup_mcpwm();
    CAN_bus.test();
    return drv.calibration() + mag.calibration() /*+ bmi270.calibration()*/;
}

void create_tasks_once() {
    static bool tasks_created = false;
    if (!tasks_created) {
        xTaskCreatePinnedToCore(controlLoop, "controlLoop", 4096, NULL, 12, &focTaskHandle, 1);  //1 and 0 are the core IDs
        if (included_bmi){
            xTaskCreatePinnedToCore(bmiLoop, "bmiLoop", 4096, NULL, 10, &bmiTaskHandle, 0);
        }
        xTaskCreatePinnedToCore(telemetryLoop, "telemetryLoop", 8192, NULL, 7, &telemetryTaskHandle, 0);
        xTaskCreatePinnedToCore(canLoop, "canLoop", 4096, NULL, 10, &canTaskHandle, 0);
        tasks_created = true;
    }
}


void controlLoop(void* arg){
    esp_task_wdt_add(NULL);  // Subscribe to watchdog

    float local_angle = 0.0f;
    float local_reducer_angle = 0.0f;
    float local_rpm = 0.0f;
    
    while(true){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        drv.read_currents_sync();  // Ensure currents are read sync with control loop
        
        if (fbk_sys == MAG) {
            mag_data temp = mag.getdata();
            local_angle = temp.angle;
            local_rpm = drv.update_rpm(local_angle); // single call
            local_reducer_angle = drv.reducer_angle(local_angle);

            xSemaphoreTake(fbk_mutex, portMAX_DELAY);
            fbk_pkg.angle = temp.angle;
            fbk_pkg.status = temp.status;
            fbk_pkg.reducer_angle = local_reducer_angle;
            xSemaphoreGive(fbk_mutex);
        }
            
        else if (fbk_sys == HALL) {
            local_angle = hall_sensor.getHallAngle();
            local_rpm = hall_sensor.getRPM(); // single call
            local_reducer_angle = local_angle;

            xSemaphoreTake(fbk_mutex, portMAX_DELAY);
            fbk_pkg.angle = local_angle;
            fbk_pkg.status = 0;  // Assuming no specific status for HALL sensor
            fbk_pkg.reducer_angle = local_reducer_angle;
            xSemaphoreGive(fbk_mutex);
        }
            
        switch (motion) {
            case MOTION_STARTUP:
            if (drv.align(local_angle).done) {
                motion = MOTION_SPINNING;
            }
            break;

            case MOTION_RAMPING:
            {
                static bool ramp_reset_done = false;
                if (!ramp_reset_done) {
                    drv.reset_spin_timer();
                    ramp_reset_done = true;
                }
                if (drv.ramp(local_angle, 2.0f, 20.0f)) {
                    ramp_reset_done = false;   // reset for next ramp
                    motion = MOTION_SPINNING;
                }
                break;
            }

            case MOTION_SPINNING:
            {
                switch (control_mode) {
                    case MODE_CLOSED_FOC:
                    switch (closed_mode) {
                            case MODE_TORQUE:
                            drv.foc_control(local_angle);
                            break;
                            case MODE_VEL:
                            drv.velocity_loop(local_rpm);
                            drv.foc_control(local_angle);
                            break;
                            case MODE_POS:
                            drv.position_loop(local_reducer_angle);
                            drv.velocity_loop(local_rpm);
                            drv.foc_control(local_angle);
                            break;
                            default: break;
                        }
                        break;
                        
                        case MODE_OPENLOOP_BEMF:
                        drv.openloop_bemf(20.0f, 50.0f);
                        break;
                        
                        case MODE_CLOSED_SINUSOIDAL:
                        drv.sinusoidal_spin(local_angle, 20.0f);
                        break;
                    default: break;
                }
                break;
            }
            case MOTION_RAMPING_DOWN:
            {
                if (drv.ramp(local_angle, 10.0f, 0.5f))   //Target freq and final freq
                    motion = MOTION_IDLE;
                break;
            }
            case MOTION_IDLE:
            {
                drv.brake();
                break;
            }
            default: break;
        }
        esp_task_wdt_reset();
    }
}

void bmiLoop(void* arg) {
    esp_task_wdt_add(NULL);
    if (included_bmi) {
        while (true) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            BMI_data temp = bmi270.getdata();
            xSemaphoreTake(bmi_mutex, portMAX_DELAY);
            bmi_pkg = temp;
            xSemaphoreGive(bmi_mutex);
            esp_task_wdt_reset();
        }
    }
    else {
        vTaskDelete(NULL);          //Delete task if BMI not included
    }
}

void canLoop(void* arg) {
    if (!CAN_enabled) {
        vTaskDelete(NULL);
        return;
    }

    uint32_t missed_rx_count = 0;
    const uint32_t MAX_MISSED_RX = 20; 

    while (true) {
        // Wait for trigger notification (e.g., from timer or control loop)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!slave_motor) {
            continue;
        }

        CAN_bus.check_and_recover();

        DRV_data td = {};
        angle_data tm = {};

        if (xSemaphoreTake(drv_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            td = drv.getdata();
            xSemaphoreGive(drv_mutex);
        }
        if (xSemaphoreTake(fbk_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            tm = fbk_pkg;
            xSemaphoreGive(fbk_mutex);
        }

        // 3. Determine system health status
        if (td.temperature < 0.0f || td.temperature > 50.0f) {
            status_flags = STATUS_OVERTEMP;
        } else if (td.PVDD < 14.0f) {
            status_flags = STATUS_UNDERVOLT;
        } else if (td.NFAULT_raw != 0) {
            status_flags = STATUS_FAULT;
        } else {
            status_flags = STATUS_OK;
        }

        slave_pkg.ID = MOTOR_ID;
        slave_pkg.status = static_cast<uint8_t>(status_flags);
        slave_pkg.mode = static_cast<uint8_t>(closed_mode);

        switch (closed_mode) {
            case MODE_POS:
                slave_pkg.value_fbk = tm.reducer_angle;
                break;
            case MODE_VEL:
                slave_pkg.value_fbk = drv.reducer_RPM(drv.get_last_rpm());
                break;
            case MODE_TORQUE:
                slave_pkg.value_fbk = 0.0f;
                break;
            default:
                slave_pkg.value_fbk = 0.0f;
                break;
        }

        if (!CAN_bus.sendData(slave_pkg, MOTOR_ID)) {
            ESP_LOGW("CAN", "Failed to send CANmotorPacket feedback (Master possibly offline)");
        }

        CANMasterPacket incoming = {};
        bool rx_ok = CAN_bus.receiveData(&incoming, master_id, pdMS_TO_TICKS(15));

        if (rx_ok && incoming.ID == MOTOR_ID) {
            missed_rx_count = 0; // Reset communication loss counter

            const bool was_idle = (motion == MOTION_IDLE);
            const ClosedMode prev_mode = closed_mode;

            switch (incoming.mode) {
                case MODE_IDLE:   
                    motion = MOTION_IDLE;  
                    break;
                case MODE_POS:    
                    drv.set_target_pos(incoming.value_cmd);    
                    closed_mode = MODE_POS;    
                    break;
                case MODE_VEL:    
                    drv.set_target_vel(incoming.value_cmd);    
                    closed_mode = MODE_VEL;    
                    break;
                case MODE_TORQUE: 
                    drv.set_target_torque(incoming.value_cmd); 
                    closed_mode = MODE_TORQUE; 
                    break;
                default:
                    ESP_LOGW("CAN", "Ignoring unknown command mode: %u", (unsigned)incoming.mode);
                    break;
            }

            if (was_idle || closed_mode != prev_mode) {
                drv.reset_spin_timer();
                drv.reset_pos_loop();
                drv.reset_vel_loop();
                drv.reset_foc();
                if (was_idle) motion = MOTION_STARTUP;
            }
        } else {
                missed_rx_count++;
            if (missed_rx_count >= MAX_MISSED_RX) {
                // SAFETY: Stop motor if master comms lost
                if (motion != MOTION_IDLE) {
                    ESP_LOGW("CAN", "Master heartbeat lost. Entering IDLE.");
                    motion = MOTION_IDLE;
                }
            }
        }
    }
}

void telemetryLoop(void* arg) {
    static char buf[300];
    if (!debug_mode_enabled) {
        vTaskDelete(NULL);          // Delete the task if debug mode is disabled
    }
    
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        DRV_data td;
        angle_data tm;
        BMI_data tb;
        
        if (xSemaphoreTake(drv_mutex, portMAX_DELAY) == pdTRUE) {
            td = drv.getdata();
            xSemaphoreGive(drv_mutex);
        } if (xSemaphoreTake(fbk_mutex, portMAX_DELAY) == pdTRUE) {
            tm = fbk_pkg;
            xSemaphoreGive(fbk_mutex);
        } if (xSemaphoreTake(bmi_mutex, portMAX_DELAY) == pdTRUE) {
            tb = bmi_pkg;
            xSemaphoreGive(bmi_mutex);
        }
        
        int len = snprintf(
            buf, sizeof(buf),
            "%d,%.3f,%.3f,%.3f,%.3f,%2d,%3d,%3d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f\n",
            MOTOR_ID,
            td.PVDD,
            td.current1,
            td.current2,
            -(td.current1 + td.current2),
            td.A,
            td.B,
            td.C,
            td.NFAULT_raw,
            td.NOCTW_raw,
            td.temperature,
            tm.angle,
            tm.reducer_angle,
            drv.get_last_rpm(),
            drv.reducer_RPM(drv.get_last_rpm())
            /*tb.accx,
            tb.accy,
            tb.accz,
            tb.gyrx,
            tb.gyry,
            tb.gyrz,
            tb.ffall*/
        );

        if (len > 0) {
            printf("%s", buf);
        }
        
        uint8_t rxbuf[64] = {0};
        int rxlen = usb_serial_jtag_read_bytes(rxbuf, sizeof(rxbuf) - 1, 0);
        
        if (rxlen > 0) {
            rxbuf[rxlen] = '\0';
            for (int i = rxlen - 1; i >= 0; i--) {
                if (rxbuf[i] == '\r' || rxbuf[i] == '\n' || rxbuf[i] == ' ')
                rxbuf[i] = '\0';
                else break;
            }

            printf("RAW CMD: [%s] len=%d\n", rxbuf, rxlen);  // add this
            fflush(stdout);
            
            char* cmd = strchr((char*)rxbuf, ':');
            if (cmd != NULL) {
                cmd++;
                if      (strncmp(cmd, "FBK:MAG_ENCODER", 15) == 0) fbk_sys = MAG;
                else if (strncmp(cmd, "FBK:HALL_SENSOR", 15) == 0) fbk_sys = HALL;
                else if (strncmp(cmd, "CTRL:OPENLOOP_BEMF", 19) == 0) control_mode = MODE_OPENLOOP_BEMF;
                else if (strncmp(cmd, "CTRL:CLOSED_FOC", 15) == 0) control_mode = MODE_CLOSED_FOC;
                else if (strncmp(cmd, "CTRL:CLOSED_SINUSOIDAL", 23) == 0) control_mode = MODE_CLOSED_SINUSOIDAL;
                else if (strcmp(cmd, "IDLE") == 0) motion = MOTION_IDLE, drv.request_foc_stop(), drv.reset_spin_timer();
                else if (strcmp(cmd, "ALIGN") == 0) drv.reset_spin_timer(), drv.reset_pos_loop(), drv.reset_vel_loop(), drv.reset_foc(), motion = MOTION_STARTUP;
                else if (strncmp(cmd, "RUN", 3) == 0){
                    int duration = atoi(strchr((char*)cmd, ':') + 1);
                    drv.set_spin_duration(duration);
                    drv.reset_spin_timer();
                    drv.reset_pos_loop();
                    drv.reset_vel_loop();
                    drv.reset_foc();
                    motion = MOTION_SPINNING;
                    printf("Spin duration set to %d seconds\n", duration);
                    printf("Spinning with mode %d and control %d\n", control_mode, closed_mode);
                    fflush(stdout);
                }
                else if (strcmp(cmd, "BRAKE") == 0) motion = MOTION_RAMPING_DOWN, drv.request_foc_stop();
                else if (strcmp(cmd, "STOP") == 0) gpio_set_level(EN_DRV, 0);
                else if (strncmp(cmd, "DUR:", 4) == 0) {
                    float duration = atof(cmd + 4);
                    drv.set_ramp_duration(duration);
                    printf("Ramp duration set to %.3f\n", duration);
                    fflush(stdout);
                }
                else if (strncmp(cmd, "PID_", 4) == 0) {
                    char prefix[10];
                    float Kp = 0.0f, Ki = 0.0f, Kd = 0.0f;
                    if (sscanf(cmd, "PID_%11[^_]_KP:%f_KI:%f_KD:%f", prefix, &Kp, &Ki, &Kd) == 4) {
                        int8_t param = -1; // Default to invalid
                        if      (strcmp(prefix, "VEL") == 0) param = 0;
                        else if (strcmp(prefix, "POS") == 0) param = 1;
                        else if (strcmp(prefix, "TORQUE") == 0) param = 2;
                        else if (strcmp(prefix, "FLUX") == 0) param = 3;
                        
                        if (param >= 0) {
                            drv.set_PID_tuning(param, Kp, Ki, Kd);
                            printf("%s PID updated: Kp=%.4f Ki=%.4f Kd=%.4f\n", prefix, Kp, Ki, Kd);
                            fflush(stdout);
                        }
                    }
                }
                else if (strncmp(cmd, "SP_", 3) == 0) {
                    float target = 0.0f;
                    if (sscanf(cmd, "SP_POS:%f", &target) == 1) {
                        drv.set_target_pos(target);
                        closed_mode = MODE_POS;
                        printf("POS Target updated: %.4f\n", target);
                        fflush(stdout);
                    }
                    else if (sscanf(cmd, "SP_VEL:%f", &target) == 1) {
                        drv.set_target_vel(target);
                        closed_mode = MODE_VEL;
                        printf("VEL Target updated: %.4f\n", target);
                        fflush(stdout);
                    }
                    else if (sscanf(cmd, "SP_TOR:%f", &target) == 1) {
                        drv.set_target_torque(target);
                        closed_mode = MODE_TORQUE;
                        printf("TOR Target updated: %.4f\n", target);
                        fflush(stdout);
                    }
                    else {
                        printf("Error parsing SP command: %s\n", cmd);
                        fflush(stdout);
                    }
                }
                else if (strcmp(cmd, "FAULT_OVERRIDE_ON") == 0) drv.set_override(true);
                else if (strcmp(cmd, "FAULT_OVERRIDE_OFF") == 0) drv.set_override(false);
                
            }
        }
    }
}

extern "C" void IRAM_ATTR mcpwm_isr(void* arg)  // IRAM_ATTR to place ISR in IRAM for faster execution
{
    MCPWM0.int_clr.timer0_tep_int_clr = 1; // Clear the MCPWM TEP interrupt flag

    static uint32_t tick = 0;
    tick++;

    BaseType_t woken = pdFALSE; // If a task was woken by this ISR (with higher priority), this will be set to true

    // Wake the fast control loop exactly on the PWM center (ONLY if task exists)
    if (tick % CONTROL_DIVIDER == 0 && focTaskHandle != NULL)
        vTaskNotifyGiveFromISR(focTaskHandle, &woken);
        
    if (tick % TELEMETRY_DIVIDER == 0 && telemetryTaskHandle != NULL)
        vTaskNotifyGiveFromISR(telemetryTaskHandle, &woken);

    if (included_bmi && tick % BMI_DIVIDER == 0 && bmiTaskHandle != NULL)
        vTaskNotifyGiveFromISR(bmiTaskHandle, &woken);

    if (tick % CAN_DIVIDER == 0 && canTaskHandle != NULL)
        vTaskNotifyGiveFromISR(canTaskHandle, &woken);

    portYIELD_FROM_ISR(woken); // If a higher priority task was woken, yield to it immediately after the ISR
}

/*
void IRAM_ATTR controlISR(void* arg)    // IRAM_ATTR to place ISR in IRAM for faster execution
{
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0); // Hardware timer interrupt clear
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0); // Rearm timer alarm

    static uint32_t tick = 0;
    tick++;

    BaseType_t woken = pdFALSE; // If a task was woken by this ISR (with higher priority), this will be set to true

    if (tick % CONTROL_DIVIDER == 0)
        vTaskNotifyGiveFromISR(focTaskHandle, &woken);
    if (tick % TELEMETRY_DIVIDER == 0)
        vTaskNotifyGiveFromISR(telemetryTaskHandle, &woken);
    if (included_bmi && tick % BMI_DIVIDER == 0)
        vTaskNotifyGiveFromISR(bmiTaskHandle, &woken);
    if (tick % CAN_DIVIDER == 0)
        vTaskNotifyGiveFromISR(canTaskHandle, &woken);

    portYIELD_FROM_ISR(woken); // If a higher priority task was woken, yield to it immediately after the ISR
}
*/



