//================================================================================================//
//                                    DRV 8302 & Peripherals                                      //
//================================================================================================//
#ifndef DRV_H
#define DRV_H 

#include "esp32_S3.h"

//================================================================================================//
//                                    MOTOR PROFILE SELECTION                                     //
//================================================================================================//
// Choose ONE profile below:
//   1 -> MOTOR_HOVERBOARD
//   2 -> MOTOR_5010_DIRECT
//   3 -> MOTOR_5010_REDUCER

#define MOTOR_PROFILE 3

#define PROFILE_HOVERBOARD   1
#define PROFILE_5010_DIRECT  2
#define PROFILE_5010_REDUCER 3

//================================================================================================//
#if (MOTOR_PROFILE == PROFILE_HOVERBOARD)

    #define CFG_MOTOR_POLE_PAIRS   15
    #define CFG_MOTOR_RESISTANCE   0.4f
    #define CFG_REDUCER_RATIO      1.0f

    // Current Loop Tuning
    #define CFG_IQ_KP 63.0f
    #define CFG_IQ_KI 30.0f
    #define CFG_IQ_KD 0.03f
    #define CFG_ID_KP 37.0f
    #define CFG_ID_KI 30.0f
    #define CFG_ID_KD 0.03f
    #define CFG_ID_LPF 0.001f
    #define CFG_IQ_LPF 0.001f

    // Velocity Loop Tuning
    #define CFG_VEL_KP  0.1f
    #define CFG_VEL_KI  0.01f
    #define CFG_VEL_KD  0.0f
    #define CFG_VEL_LPF 0.05f

    // Position Loop Tuning
    #define CFG_POS_KP  20.0f
    #define CFG_POS_KI  0.0f
    #define CFG_POS_KD  0.0f
    #define CFG_POS_LPF 0.1f

#elif (MOTOR_PROFILE == PROFILE_5010_DIRECT)

    #define CFG_MOTOR_POLE_PAIRS   7
    #define CFG_MOTOR_RESISTANCE   0.270f
    #define CFG_REDUCER_RATIO      1.0f

    // Current Loop Tuning
    #define CFG_IQ_KP 0.005f
    #define CFG_IQ_KI 3.0f
    #define CFG_IQ_KD 0.0f
    #define CFG_ID_KP 0.005f
    #define CFG_ID_KI 3.0f
    #define CFG_ID_KD 0.0f
    #define CFG_ID_LPF 0.8f
    #define CFG_IQ_LPF 0.8f

    // Velocity Loop Tuning
    #define CFG_VEL_KP  0.002f
    #define CFG_VEL_KI  0.008f
    #define CFG_VEL_KD  0.0f
    #define CFG_VEL_LPF 0.8f

    // Position Loop Tuning
    #define CFG_POS_KP  20.0f
    #define CFG_POS_KI  0.0f
    #define CFG_POS_KD  0.0f
    #define CFG_POS_LPF 0.8f

#elif (MOTOR_PROFILE == PROFILE_5010_REDUCER)

    #define CFG_MOTOR_POLE_PAIRS   7
    #define CFG_MOTOR_RESISTANCE   0.270f
    #define CFG_REDUCER_RATIO      15.0f

    // Current Loop Tuning
    #define CFG_IQ_KP 0.1f
    #define CFG_IQ_KI 10.0f
    #define CFG_IQ_KD 0.0f
    #define CFG_ID_KP 0.1f
    #define CFG_ID_KI 10.0f
    #define CFG_ID_KD 0.0f
    #define CFG_ID_LPF 0.05f
    #define CFG_IQ_LPF 0.05f

    // Velocity Loop Tuning
    #define CFG_VEL_KP  0.1f
    #define CFG_VEL_KI  0.01f
    #define CFG_VEL_KD  0.0f
    #define CFG_VEL_LPF 0.1f

    // Position Loop Tuning
    #define CFG_POS_KP  20.0f
    #define CFG_POS_KI  0.0f
    #define CFG_POS_KD  0.0f
    #define CFG_POS_LPF 0.1f

#else
    #error "Invalid MOTOR_PROFILE selected! Please select a valid profile at the top of drv.h"
#endif

//================================================================================================//
//                                    STRUCTURES & ENUMS                                         //
//================================================================================================//

enum DRV_Error {
    DRV_OK = 0,
    DRV_OVERVOLTAGE = -1,
    DRV_FAULT = -2,
    DRV_WARNING = -3,
    DRV_LATCHED = -4
};

struct AlignOffset {
    bool  done;
    float elec_offset;
};

typedef struct {
    float PVDD;
    float current1;
    float current2;
    uint16_t A;
    uint16_t B;
    uint16_t C;
    uint8_t NFAULT_raw;
    uint8_t NOCTW_raw;
    float temperature;
} DRV_data;


//================================================================================================//
//                                        DRV CLASS                                               //
//================================================================================================//

class DRV {
    public:
        int init();
        int calibration();
        DRV_data getdata();
        int fault_detection();

        float real_RPM(float angle_deg);
        float reducer_angle(float motor_angle);
        void setup_mcpwm();
        bool svpwm(float v_a, float v_b, float v_c);
        AlignOffset align(float angle);
        bool ramp(float angle, float init_freq, float obj_freq);
        bool sinusoidal_spin(float fbk_angle, float freq);
        bool foc_control(float fbk_angle);

        float position_loop(float fbk_angle_deg);
        float velocity_loop(float fbk_rpm);

        void request_foc_stop() { foc_stop = true; }
        void reset_foc() {
            i_d_filt = 0.0f; i_q_filt = 0.0f;
            errorq_prev = 0.0f; errord_prev = 0.0f;
            errorq_integral = 0.0f; errord_integral = 0.0f;
            foc_initialized = false;
            foc_stop = false;
        }
        void reset_pos_loop() {
            pos_error_integral = 0.0f;
            pos_error_prev = 0.0f;
            pos_error_filt = 0.0f;
            pos_initialized = false;
            pos_prev_time = 0.0f;
        }
        void reset_vel_loop() {
            vel_error_integral = 0.0f;
            vel_error_prev     = 0.0f;
            vel_initialized = false;
            vel_prev_time = 0.0f;
        }
        bool openloop_bemf(float target_freq, float duty);
        void brake();
        void idle();
        void test_working_trans();

        void set_override(bool state){
            fault_override = state;
            if (state)
                printf("DRV FAULT OVERRIDE ENABLED\n");
            else
                printf("DRV FAULT OVERRIDE DISABLED\n");
        }

        float get_last_rpm() const { return last_rpm; }
        float update_rpm (float angle){ last_rpm = real_RPM(angle); return last_rpm;}
        float getPVDD(int PVDD_raw){
            float R_frac = 11;  // 100k + 10k / 10k
            return (float)PVDD_raw * 3.3 / 4095 * R_frac + V_offset;
        }

        void read_currents_sync() {
            SO1_raw = adc1_get_raw(SO1);
            SO2_raw = adc1_get_raw(SO2);
        }

        void set_MOC(bool state){
            gpio_set_level(MOC, state ? 1 : 0);
        }

        void set_spin_duration(float duration_s){ spin_duration = duration_s; }
        void set_PID_tuning(uint8_t parameter, float Kp, float Ki, float Kd){
            switch (parameter) {
                case 0: Vel_Kp = Kp; Vel_Ki = Ki; Vel_Kd = Kd; break;
                case 1: Pos_Kp = Kp; Pos_Ki = Ki; Pos_Kd = Kd; break;
                case 2: Iq_Kp  = Kp; Iq_Ki = Ki; Iq_Kd = Kd; break;
                case 3: Id_Kp  = Kp; Id_Ki = Ki; Id_Kd = Kd; break;
                default: break;
            }
        }
        void set_target_pos(float pos)       { target_pos = pos; }
        void set_target_vel(float vel)       { target_vel = vel; }
        void set_target_torque(float torque) { target_current = torque; }

        void reset_spin_timer() {
            spin_timer_started = false;
            spin_elapsed = 0.0f;
            theta_reset = true;
            phase_accumulator = 0.0f;     
            electrical_freq = 2.0f;        
            rpm_last_time = 0;
            ramp_active = false;
            align_time = 0.0f;
        }
        void set_ramp_duration(float duration_s){ ramp_duration = duration_s; }

        float reducer_RPM(float motor_RPM){ return motor_RPM / reducer_ratio; }

    private: 
        // DRV data
        uint16_t SO1_raw = 0;   // Output current 1
        uint16_t SO2_raw = 0;   // Output current 2
        uint16_t NFAULT_raw = 0;
        uint16_t NOCTW_raw = 0;
        int PVDD_raw = 0;
        float V_offset = 1.1f;  // Voltage offset

        // DRV parameters
        float R_shunt = 0.01;
        float voltage_step = 3.3 / 4095;
        float overvoltage = 16;
        uint16_t SO1_offset = 2048; // Default to mid-range (1.65V)
        uint16_t SO2_offset = 2048;

        // Motor parameters
        static constexpr int MOTOR_POLE_PAIRS   = CFG_MOTOR_POLE_PAIRS;
        static constexpr float MOTOR_RESISTANCE = CFG_MOTOR_RESISTANCE;
        static constexpr float reducer_ratio    = CFG_REDUCER_RATIO;
        
        // DRV current sensing
        float current_gain = 10.0; 

        // Voltage sensing
        float voltage_gain = 11.0f; // 100k + 10k / 10k

        // External sensors data
        int NTC_raw = 0;
        uint32_t last_ntc_read = 0;
        float last_temperature = 0.0f;
        
        // Fault handling
        uint16_t latch_counter = 0;
        bool fault_override = false;
        uint8_t nfault_glitch_count = 0; 

        // Alignment and offset
        float align_time = 0.0;
        float ALIGN_DURATION = 0.2f;   // 200 ms

        // Waveform generation
        uint32_t PWM_freq = 40000;     // 40 kHz carrier frequency for MCPWM
        float phase_accumulator = 0;
        float modulation_index = 1.0f; // Amplitude (0.0 to 1.0)
        float electrical_freq = 2;     // Motor Frequency (Hz)
        float elec_offset = 0.0f; 
        float theta_reset = true;

        // Ramping control
        float ramp_rate = 50.0f;       // How fast to change frequency (Hz/s)
        float ramp_active = false;
        float ramp_target_freq = 0.0f;
        float ramp_duration = .5f;
        float dt = 0.000050f;          // Elapsed time (assuming 50us)

        // Spinning control
        float spin_elapsed = 0.0f;
        float spin_duration = 0.0f;
        float spin_start_time = 0.0f;
        bool spin_timer_started = false;

        // Reducer angle
        float last_angle = 0.0f;
        float reducer = 0.0f;
        bool initialized = false;

        // RPM calculation
        float   rpm_last_angle  = 0.0f;
        int64_t rpm_last_time   = 0;
        float   rpm_filtered    = 0.0f;
        float last_rpm = 0.0f;
        
        float CLOSED_LOOP_LPF = 1.0f;  // 1.0 = no filtering
        static constexpr float RPM_UNWRAP_THR = 180.0f;
        static constexpr int64_t RPM_MIN_DT_US = 50;

        // Tunings (Auto-configured by MOTOR_PROFILE macro)
        float Vel_Kp  = CFG_VEL_KP;    
        float Vel_Ki  = CFG_VEL_KI;  
        float Vel_Kd  = CFG_VEL_KD;
        float Vel_LPF = CFG_VEL_LPF;   
        float target_vel = 0.0f;

        float Pos_Kp  = CFG_POS_KP;  
        float Pos_Ki  = CFG_POS_KI;    
        float Pos_Kd  = CFG_POS_KD; 
        float Pos_LPF = CFG_POS_LPF;  
        float target_pos = 0.0f;

        float Id_LPF = CFG_ID_LPF;
        float Iq_LPF = CFG_IQ_LPF;   
        float Iq_Kp  = CFG_IQ_KP;   float Iq_Ki = CFG_IQ_KI;   float Iq_Kd = CFG_IQ_KD;
        float Id_Kp  = CFG_ID_KP;   float Id_Ki = CFG_ID_KI;   float Id_Kd = CFG_ID_KD;

        float i_d_filt = 0.0f;
        float i_q_filt = 0.0f;
        float target_current = 0.0f;
        float errorq_prev = 0.0f;
        float errorq_integral = 0.0f;
        float errord_prev = 0.0f;
        float errord_integral = 0.0f;
        float foc_prev_time = 0.0f;
        bool foc_initialized = false;
        bool just_aligned = true;
        bool foc_stop = false;

        // Reducer limits
        float reducer_max_angle = 90.0f;
        float reducer_min_angle = -90.0f;
        bool reducer_limits_enabled = true;

        // Position loop state
        float pos_error_integral = 0.0f;
        float pos_error_filt = 0.0f;
        float pos_error_prev = 0.0f;
        float pos_prev_time = 0.0f;
        bool pos_initialized = false;

        // Velocity loop state
        float vel_error_integral = 0.0f;
        float fbk_rpm_filt = 0.0f;
        float vel_error_prev = 0.0f;
        float vel_prev_time = 0.0f;
        bool vel_initialized = false;

        // Velocity loop output clamp (Amps)
        static constexpr float VEL_OUT_MAX = 4.0f;
        // Position loop output clamp (RPM)
        static constexpr float POS_OUT_MAX = 500.0f;
    };

class NTC_sensor {
    public:
        float getTemperature(int NTC_raw){
            float ratio = (float)NTC_raw / (4095.0f - (float)NTC_raw);
            return 1.0f / (INV_T0 + INV_B * logf(ratio)) - 273.15f - offset;
        }
        void change_offset(int mod){offset += (float)mod;}

    private:
        static constexpr int B  = 3380;
        static constexpr float T0 = 298.15f;  // 25 + 273.15

        static constexpr float INV_T0 = 1.0f / T0;
        static constexpr float INV_B = 1.0f / (float)B;

        float offset = 9.0f;
};

#endif