#include <micro_ros_arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int8.h>
#include <geometry_msgs/msg/vector3.h>
#include <esp32-hal-timer.h>

//==================================================
// PINS CONFIGURATION
//==================================================
#define SDA_PIN 8
#define SCL_PIN 9

#define RPWM 6
#define LPWM 7

#define TRIG 12
#define ECHO 13

const int stepPinL = 40;
const int dirPinL  = 41;
const int enPinL   = 42;
const int stepPinR = 36;
const int dirPinR  = 37;
const int enPinR   = 38;

//==================================================
// CONSTANTS & TARGETS
//==================================================
const uint32_t IMU_PUBLISH_RATE = 100;
const uint32_t IMU_PERIOD_MS = 1000 / IMU_PUBLISH_RATE;

const float MIN_H = 7.0;
const float MAX_H = 22.0;
const float TARGET_TOL = 0.7;
const float SLOW_ZONE  = 3.0;
const float BRAKE_ZONE = 2.0;

// Watchdog banh xe: mat lenh /wheel_speed qua nguong nay -> dung steppers
const uint32_t WHEEL_TIMEOUT_MS = 500;

// [WHEEL_ODOM] Chu ky publish so xung tich luy (20 Hz)
const uint32_t TICKS_PERIOD_MS = 50;

enum {
    IDLE = 0,
    MOVING_UP = 1,
    MOVING_DOWN = 2,
    TARGET_REACHED = 3,
    MIN_LIMIT = 4,
    MAX_LIMIT = 5
};

//==================================================
// MICRO-ROS CONNECTION STATE MACHINE
//==================================================
enum AgentState {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};
AgentState agent_state = WAITING_AGENT;

// Chay 1 doan lenh moi N ms (khong block)
#define EXECUTE_EVERY_N_MS(MS, X) do { \
    static volatile int64_t init_t = -1; \
    if (init_t == -1) { init_t = millis(); } \
    if (millis() - init_t > (MS)) { X; init_t = millis(); } \
} while (0)

// Tra ve false neu mot buoc init that bai
#define RCCHECK_BOOL(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { return false; } }

//==================================================
// STATE VARIABLES
//==================================================
Adafruit_BNO055 bno(55, 0x28, &Wire);

volatile float target_h = 15.0;
float current_h = 15.0;
int8_t cylinder_status = IDLE;
bool command_active = false;
bool hold_position = false;

// Watchdog banh xe
volatile unsigned long last_wheel_ms = 0;

hw_timer_t *timerL = NULL;
hw_timer_t *timerR = NULL;
volatile bool stepL = false;
volatile bool stepR = false;
volatile float freqL = 0;
volatile float freqR = 0;
volatile int dirL = 1;
volatile int dirR = 1;

// [WHEEL_ODOM] So xung tich luy moi banh (co dau theo chieu quay).
// Dem trong ISR onTimer, doc trong connected_tasks de publish.
volatile int32_t pulseL = 0;
volatile int32_t pulseR = 0;

//==================================================
// MICRO-ROS OBJECTS
//==================================================
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_publisher_t imu_pub;
rcl_publisher_t pub_state;
rcl_publisher_t pub_status;
rcl_publisher_t pub_wheel_ticks;   // [WHEEL_ODOM]

rcl_subscription_t sub_cmd;
rcl_subscription_t sub_wheel_speed;

sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Float32 cmd_msg;
std_msgs__msg__Float32 state_msg;
std_msgs__msg__Int8 status_msg;
geometry_msgs__msg__Vector3 wheel_msg;
geometry_msgs__msg__Vector3 ticks_msg;   // [WHEEL_ODOM]

//==================================================
// STEPPER INTERRUPTS
//==================================================
// [WHEEL_ODOM] Dem 1 xung moi canh LEN (stepL doi tu false->true).
// Mot chu ky toggle (false->true->false) = 1 xung step. Nhan dau dirL/dirR
// de xung co huong: tien thi cong, lui thi tru.
void IRAM_ATTR onTimerL() {
    if (freqL <= 0.1) return;
    stepL = !stepL;
    digitalWrite(stepPinL, stepL);
    if (stepL) pulseL += dirL;   // [WHEEL_ODOM]
}

void IRAM_ATTR onTimerR() {
    if (freqR <= 0.1) return;
    stepR = !stepR;
    digitalWrite(stepPinR, stepR);
    if (stepR) pulseR += dirR;   // [WHEEL_ODOM]
}

void updateTimer(hw_timer_t *timer, float freq) {
    if (freq <= 0.1) return;
    uint64_t period_us = (uint64_t)(1000000.0 / (2.0 * freq));
    timerAlarm(timer, period_us, true, 0);
}

//==================================================
// DOC SIEU AM (BLOCKING - giong ban da verify)
// Chi goi khi dang nang-ha (xe dung yen) -> khong giat stepper.
//==================================================
float readHeightRaw() {
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    long duration = pulseIn(ECHO, HIGH, 30000);
    if (duration <= 0) return current_h;   // mau loi -> giu gia tri cu
    return duration * 0.034f / 2.0f;
}

float readHeight() {
    float sum = 0, min_v = 999, max_v = 0;
    for (int i = 0; i < 7; i++) {
        float v = readHeightRaw();
        sum += v;
        if (v < min_v) min_v = v;
        if (v > max_v) max_v = v;
        delay(2);
    }
    sum = sum - min_v - max_v;
    return sum / 5.0f;
}

//==================================================
// MOTOR FUNCTIONS (xy lanh nang ha)
//==================================================
void motorStop() {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
}

void motorBrake() {
    analogWrite(RPWM, 255);
    analogWrite(LPWM, 255);
}

void motorUp(int pwm) {
    pwm = constrain(pwm, 0, 255);
    analogWrite(RPWM, 0);
    analogWrite(LPWM, pwm);
}

void motorDown(int pwm) {
    pwm = constrain(pwm, 0, 255);
    analogWrite(RPWM, pwm);
    analogWrite(LPWM, 0);
}

int computePWM(float error) {
    float a = fabs(error);
    if(a > 6) return 160;
    if(a > 4) return 120;
    if(a > 2) return 90;
    return 60;
}

// PWM toi thieu de xy lanh con du luc thang tai/ma sat van/seal khi xuong.
// Duoi muc nay dong co/van quay nhung khong sinh du luc -> "dung giua duong"
// (day la trieu chung ban gap o 16cm/target 20cm). Tang/giam so theo tai
// trong thuc te cua xy lanh ban dang dung.
const int MIN_EFFECTIVE_PWM_DOWN = 90;

//==================================================
// DUNG AN TOAN: goi khi mat ket noi agent
//==================================================
void safe_stop_all() {
    freqL = 0;            // onTimer se ngung dap xung -> stepper dung
    freqR = 0;
    motorStop();          // cat PWM xy lanh
    command_active = false;
    hold_position = true;
}

//==================================================
// ROS CALLBACKS
//==================================================
void cmd_callback(const void * msgin) {
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    float t = msg->data;
    if(t > MAX_H) t = MAX_H;
    if(t < MIN_H) t = MIN_H;
    target_h = t;
    command_active = true;
    hold_position = false;
}

void wheel_speed_callback(const void *msgin) {
    const geometry_msgs__msg__Vector3 *m = (const geometry_msgs__msg__Vector3 *)msgin;
    if (m->x >= 0) {
        dirL = 1;   digitalWrite(dirPinL, HIGH);  freqL = m->x;
    } else {
        dirL = -1;  digitalWrite(dirPinL, LOW);   freqL = -m->x;
    }
    if (m->y >= 0) {
        dirR = 1;   digitalWrite(dirPinR, HIGH);  freqR = m->y;
    } else {
        dirR = -1;  digitalWrite(dirPinR, LOW);   freqR = -m->y;
    }
    updateTimer(timerL, freqL);
    updateTimer(timerR, freqR);
    last_wheel_ms = millis();   // <-- nuoi watchdog
}

//==================================================
// TAO / HUY ENTITY MICRO-ROS  (chia rieng de tai ket noi)
//==================================================
bool create_entities() {
    allocator = rcl_get_default_allocator();

    RCCHECK_BOOL(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK_BOOL(rclc_node_init_default(&node, "agv_main_node", "", &support));

    RCCHECK_BOOL(rclc_publisher_init_default(
        &imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data"));
    RCCHECK_BOOL(rclc_publisher_init_default(
        &pub_state, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/cylinder_state"));
    RCCHECK_BOOL(rclc_publisher_init_default(
        &pub_status, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "/cylinder_status"));

    // [WHEEL_ODOM] Publisher so xung tich luy (x=pulseL, y=pulseR, z=ms)
    RCCHECK_BOOL(rclc_publisher_init_default(
        &pub_wheel_ticks, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/wheel_ticks"));

    RCCHECK_BOOL(rclc_subscription_init_default(
        &sub_cmd, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/cylinder_cmd"));
    RCCHECK_BOOL(rclc_subscription_init_default(
        &sub_wheel_speed, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/wheel_speed"));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK_BOOL(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK_BOOL(rclc_executor_add_subscription(
        &executor, &sub_cmd, &cmd_msg, &cmd_callback, ON_NEW_DATA));
    RCCHECK_BOOL(rclc_executor_add_subscription(
        &executor, &sub_wheel_speed, &wheel_msg, &wheel_speed_callback, ON_NEW_DATA));

    // Dong bo thoi gian cho timestamp IMU
    rmw_uros_sync_session(1000);

    last_wheel_ms = millis();   // reset watchdog khi vua ket noi
    return true;
}

void destroy_entities() {
    // Khong block khi agent da bien mat
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rclc_executor_fini(&executor);
    rcl_publisher_fini(&imu_pub, &node);
    rcl_publisher_fini(&pub_state, &node);
    rcl_publisher_fini(&pub_status, &node);
    rcl_publisher_fini(&pub_wheel_ticks, &node);   // [WHEEL_ODOM]
    rcl_subscription_fini(&sub_cmd, &node);
    rcl_subscription_fini(&sub_wheel_speed, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

//==================================================
// SETUP (chi khoi tao PHAN CUNG mot lan; micro-ROS do state machine lo)
//==================================================
void setup() {
    Serial.begin(115200);
    delay(2000);

    pinMode(RPWM, OUTPUT);   pinMode(LPWM, OUTPUT);
    pinMode(TRIG, OUTPUT);   pinMode(ECHO, INPUT);
    pinMode(stepPinL, OUTPUT); pinMode(stepPinR, OUTPUT);
    pinMode(dirPinL, OUTPUT);  pinMode(dirPinR, OUTPUT);
    pinMode(enPinL, OUTPUT);   pinMode(enPinR, OUTPUT);

    motorStop();
    digitalWrite(enPinL, LOW);  digitalWrite(enPinR, LOW);
    digitalWrite(dirPinL, HIGH); digitalWrite(dirPinR, HIGH);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    if (!bno.begin()) {
        while (1) { delay(100); }
    }
    bno.setExtCrystalUse(true);

    timerL = timerBegin(1000000);
    timerR = timerBegin(1000000);
    timerAttachInterrupt(timerL, &onTimerL);
    timerAttachInterrupt(timerR, &onTimerR);
    timerAlarm(timerL, 1000, true, 0);
    timerAlarm(timerR, 1000, true, 0);

    set_microros_transports();

    // Khoi tao message IMU MOT LAN (tranh ro ri bo nho khi tai ket noi)
    sensor_msgs__msg__Imu__init(&imu_msg);
    static char frame_id[] = "imu_link";
    imu_msg.header.frame_id.data = frame_id;
    imu_msg.header.frame_id.size = strlen(frame_id);
    imu_msg.header.frame_id.capacity = strlen(frame_id) + 1;

    imu_msg.orientation_covariance[0] = 0.001;  imu_msg.orientation_covariance[4] = 0.001;  imu_msg.orientation_covariance[8] = 0.001;
    imu_msg.angular_velocity_covariance[0] = 0.0001; imu_msg.angular_velocity_covariance[4] = 0.0001; imu_msg.angular_velocity_covariance[8] = 0.0001;
    imu_msg.linear_acceleration_covariance[0] = 0.01; imu_msg.linear_acceleration_covariance[4] = 0.01; imu_msg.linear_acceleration_covariance[8] = 0.01;

    agent_state = WAITING_AGENT;
}

//==================================================
// CAC TAC VU KHI DA KET NOI (publish + xy lanh)
//==================================================
void connected_tasks() {
    uint32_t current_time = millis();
    static uint32_t last_imu_pub = 0;
    static uint32_t last_cylinder_pub = 0;
    static uint32_t last_ticks_pub = 0;   // [WHEEL_ODOM]

    // --- Watchdog banh xe: mat lenh -> dung steppers ---
    if (current_time - last_wheel_ms > WHEEL_TIMEOUT_MS) {
        freqL = 0;
        freqR = 0;
    }

    // --- [WHEEL_ODOM] PUBLISH SO XUNG TICH LUY (20 Hz) ---
    // Doc pulseL/pulseR atomically (chan ngat ISR trong tich tac doc).
    if (current_time - last_ticks_pub >= TICKS_PERIOD_MS) {
        last_ticks_pub = current_time;
        noInterrupts();
        int32_t pl = pulseL;
        int32_t pr = pulseR;
        interrupts();
        ticks_msg.x = (double)pl;            // xung tich luy banh trai (co dau)
        ticks_msg.y = (double)pr;            // xung tich luy banh phai
        ticks_msg.z = (double)current_time;  // ms ESP32 (de phia ROS tinh dt neu can)
        rcl_publish(&pub_wheel_ticks, &ticks_msg, NULL);
    }

    // --- TAC VU 1: PUBLISH IMU (100 Hz) ---
    if (current_time - last_imu_pub >= IMU_PERIOD_MS) {
        last_imu_pub = current_time;

        int64_t time_ns = rmw_uros_epoch_nanos();
        imu_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000ULL);
        imu_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000ULL);

        imu::Quaternion quat = bno.getQuat();
        imu_msg.orientation.w = quat.w(); imu_msg.orientation.x = quat.x();
        imu_msg.orientation.y = quat.y(); imu_msg.orientation.z = quat.z();

        imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        imu_msg.angular_velocity.x = gyro.x() * DEG_TO_RAD;
        imu_msg.angular_velocity.y = gyro.y() * DEG_TO_RAD;
        imu_msg.angular_velocity.z = gyro.z() * DEG_TO_RAD;

        imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        imu_msg.linear_acceleration.x = accel.x();
        imu_msg.linear_acceleration.y = accel.y();
        imu_msg.linear_acceleration.z = accel.z();

        rcl_publish(&imu_pub, &imu_msg, NULL);
    }

    // --- TAC VU 2: DOC SIEU AM + PUBLISH TRANG THAI XY LANH (10 Hz) ---
    // Doc current_h moi 100ms, NHUNG chi khi xe DUNG YEN (freqL=freqR=0).
    // - Luc nang-ha xe luon dung yen -> van doc binh thuong, dieu khien dung.
    // - Luc xe DANG CHAY: bo qua doc (blocking ~150ms se lam tre publish
    //   /wheel_ticks va watchdog). Khi chay khong can do chieu cao.
    if (current_time - last_cylinder_pub >= 100) {
        last_cylinder_pub = current_time;
        bool moving = (freqL > 0.1f) || (freqR > 0.1f);
        if (!moving) {
            current_h = readHeight();          // blocking, chi khi dung yen
        }
        state_msg.data = current_h;
        status_msg.data = cylinder_status;
        rcl_publish(&pub_state, &state_msg, NULL);
        rcl_publish(&pub_status, &status_msg, NULL);
    }

    // --- DIEU KHIEN XY LANH NANG-HA ---
    if (!command_active || hold_position) {
        motorStop();
    } else {
        float error = target_h - current_h;
        float abs_error = fabs(error);

        if (abs_error < TARGET_TOL) {
            motorBrake();
            cylinder_status = TARGET_REACHED;
            command_active = false;
            hold_position = true;
        }
        else {
            if (error > 0) {
                // Chieu LEN: luon full cong suat (255) de du luc nang hang,
                // khong giam theo SLOW_ZONE/computePWM nua.
                if (current_h >= MAX_H) {
                    motorStop();
                    cylinder_status = MAX_LIMIT;
                    command_active = false;
                } else {
                    motorUp(255);
                    cylinder_status = MOVING_UP;
                }
            }
            else if (error < 0) {
                int pwm = computePWM(error);
                if (abs_error < SLOW_ZONE) pwm = 60;

                if (current_h <= MIN_H) {
                    motorStop();
                    cylinder_status = MIN_LIMIT;
                    command_active = false;
                } else {
                    int pwm_down = pwm * 0.35;
                    if (pwm_down < MIN_EFFECTIVE_PWM_DOWN) pwm_down = MIN_EFFECTIVE_PWM_DOWN;
                    motorDown(pwm_down);
                    cylinder_status = MOVING_DOWN;
                }
            }
        }
    }
}

//==================================================
// MAIN LOOP - MAY TRANG THAI KET NOI
//==================================================
void loop() {
    switch (agent_state) {

        case WAITING_AGENT:
            // Chua co agent: giu xe dung an toan, thu ping moi 500ms
            safe_stop_all();
            EXECUTE_EVERY_N_MS(500,
                agent_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                              ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;

        case AGENT_AVAILABLE:
            // Agent vua xuat hien: tao entity moi (dong bo voi launch moi)
            agent_state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
            if (agent_state == WAITING_AGENT) {
                destroy_entities();
            }
            break;

        case AGENT_CONNECTED:
            // Theo doi ket noi moi 300ms; mat ping -> chuyen sang ngat
            EXECUTE_EVERY_N_MS(300,
                agent_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                              ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (agent_state == AGENT_CONNECTED) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
                connected_tasks();
            }
            break;

        case AGENT_DISCONNECTED:
            // Mat agent (vd ban tat launch cu): dung xe, huy session cu
            safe_stop_all();
            destroy_entities();
            agent_state = WAITING_AGENT;
            break;
    }
}