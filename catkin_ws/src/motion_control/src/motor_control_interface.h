#ifndef PRINT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct motion_cmd_t{
    uint32_t state;
    uint32_t mode;
    uint32_t foot;
    uint32_t forceaid;
    uint32_t max_force;
    int32_t max_position;
    int32_t zero_position;
    int32_t preload_position;
    int32_t max_velocity;
    int32_t nset_acc;
    float max_pot;
    float pid_kp;
    float pid_ki;
    int32_t pid_umax;
    int32_t pid_umin;
}motion_cmd_t;

void motor_init();
void motor_powerdown();
void motor_sleep();
void motor_stop();
void motor_start(u_int16_t motor_start_type);
void load_parameter(motion_cmd_t motion_cmd_para_data);

#ifdef __cplusplus
}
#endif

#define PRINT_H
#endif
