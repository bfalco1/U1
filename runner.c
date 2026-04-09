#include "runner.h"
#include "utils.h"

// declare constants
uint8_t TOOCLOSE_DISTANCE = 33; // 40 mm
uint8_t DESIRED_DISTANCE = 40; // 60 mm

extern uint8_t num_robots;
extern uint8_t current_runner;

// declare motion variable type
typedef enum {
    STOP,
    FORWARD,
    LEFT,
    RIGHT
} motion_t;

// declare state variable type
typedef enum {
    ORBIT_TOOCLOSE,
    ORBIT_NORMAL,
} orbit_state_t;

// variables from original orbit code 
motion_t cur_motion;
orbit_state_t orbit_state;
uint8_t orbit_reverse_direction = 0;

uint8_t new_message;
distance_measurement_t dist;
message_t msg;

// variables for leap frog
uint8_t rx_kilo_id; // id of bot that sent message
uint8_t front_kilo_id; // id of the bot in the front
uint8_t second_kilo_id; // id of bot behind front one; bot used to know when to stop runner at the front
uint8_t prev_second_dist; // the previous distance from the runner to the second bot
uint8_t dist_val; // distance from the message received
uint8_t stop_flag; // flag to stop runner
uint8_t switch_sent_flag; // flag to initialize switch == set compute new current_runner and set in message
uint8_t cur_target_kilo_id; // id of bot runner is currently orbiting
uint8_t current_runner_local; // should be same as current_runner until switch. It doesn't work if I don't separate it into current_runner and current_runner_local. I think it's because I'm getting current_runner from main, but I'm also not really sure still

// map of distances between kilobots and front kilobot
uint16_t first_second_distance;
uint16_t first_runner_dist;
uint16_t second_runner_dist;
uint8_t target_runner_dist;

// variables for num_robots = 2
uint8_t orbit_switch_ct;

// function to set new motion
// UNCHANGED
void set_motion(motion_t new_motion) {
    if (cur_motion != new_motion) {
        cur_motion = new_motion;
        switch(cur_motion) {
            case STOP:
                set_motors(0,0);
                break;
            case FORWARD:
                spinup_motors();
                set_motors(kilo_straight_left, kilo_straight_right);
                break;
            case LEFT:
                spinup_motors();
                set_motors(kilo_turn_left, 0); 
                break;
            case RIGHT:
                spinup_motors();
                set_motors(0, kilo_turn_right); 
                break;
        }
    }
}

// Added num_robots = 2 case orbit switch count
void orbit_normal() {
    if (target_runner_dist < TOOCLOSE_DISTANCE) {
        orbit_state = ORBIT_TOOCLOSE;
    } else {
        if (target_runner_dist < DESIRED_DISTANCE){
            if (num_robots == 2 && cur_motion != LEFT){
                orbit_switch_ct++;
                set_color(RGB(1,1,1));
                delay(20);
                set_color(RGB(0,0,0));
            }
            if(orbit_reverse_direction)
                set_motion(RIGHT);
            else
                set_motion(LEFT);
        } else{
            if(orbit_reverse_direction)
                set_motion(LEFT);
            else
                set_motion(RIGHT);
        }
    }
}

// UNCHANGED
void orbit_tooclose() {
    if (target_runner_dist >= DESIRED_DISTANCE )
        orbit_state = ORBIT_NORMAL;
    else
        set_motion(FORWARD);
}

void runner_setup(){
    // Runner is this bot
    current_runner_local = kilo_uid;

    // Reset zero values
    stop_flag = 0;
    switch_sent_flag = 0;
    prev_second_dist = 0;
    target_runner_dist = 0;
    new_message = 0;

    // Modulo math. On the first pass with three robots it should go: 1, 2, 1. With init runner being 0. 
    cur_target_kilo_id = (current_runner + 1) % num_robots;
    front_kilo_id = (current_runner - 1 + num_robots) % num_robots;
    second_kilo_id = (front_kilo_id - 1 + num_robots) % num_robots;

    first_second_distance = UINT8_MAX;
    first_runner_dist = UINT8_MAX;
    second_runner_dist = UINT8_MAX;

    cur_motion = STOP;
    orbit_state = ORBIT_NORMAL;

    // Set both message parts
    msg.type = NORMAL;
    msg.data[0] = kilo_uid;
    msg.data[1] = current_runner_local;
    msg.crc = message_crc(&msg);

    // num_robots = 2
    orbit_switch_ct = 0;
    set_color(RGB(1,1,1));
    delay(20);
    set_color(RGB(0,0,0));
}

void stop(){
    set_motion(STOP);
    if (switch_sent_flag == 0){
        current_runner_local = (current_runner_local + 1) % num_robots;
        msg.data[1] = current_runner_local;
        msg.crc = message_crc(&msg);
        switch_sent_flag = 1;
        orbit_reverse_direction = !orbit_reverse_direction;
    }
}

uint16_t last_error = UINT16_MAX;
uint32_t last_second_message = 0;
uint8_t max_second_dist = UINT8_MAX;

void runner_loop(){
    if(stop_flag){
        set_motion(STOP);
        return;
    }

    // Update distance estimate with every message
    if (new_message) {
        new_message = 0;
        dist_val = estimate_distance(&dist);

        // Update target_runner_dist if message is from target bot
        if (rx_kilo_id == cur_target_kilo_id){
            target_runner_dist = dist_val;
            set_color(id_to_color(kilo_uid, num_robots));
            delay(50);
            set_color(id_to_color(cur_target_kilo_id, num_robots));
        }

        if(rx_kilo_id == front_kilo_id){
            first_second_distance = dist_val;
            first_runner_dist = dist_val;
        }

        if(rx_kilo_id == second_kilo_id){
            second_runner_dist = dist_val;
            last_second_message = kilo_ticks;
        }

        if (num_robots != 2){
            // If incoming dist value is less than my previous distance and that distance isn't from the bot runner is currently orbiting, switch the target bot and blink BLUE
            if (dist_val < target_runner_dist && rx_kilo_id != cur_target_kilo_id){
                target_runner_dist = dist_val;
                cur_target_kilo_id = rx_kilo_id;
            }

            if(kilo_ticks - last_second_message > 50){
                for(uint8_t i=0;i<3;i++){
                    set_color(RGB(1,1,1));
                    delay(20);
                    set_color(0);
                    delay(20);
                }
                // stop();
                return;
            }
            if(cur_target_kilo_id == front_kilo_id){
                float sum = (first_second_distance + first_runner_dist)*1.18;
                // float sum = (first_second_distance + first_runner_dist)*1.15;
                float error;
                if(second_runner_dist > sum)
                    error = second_runner_dist - sum;
                else
                    error = sum - second_runner_dist;                    
                if((error > last_error) && (error < 10)){
                // if((error > last_error) && (error < 15)){
                    stop();
                    return;
                }
                last_error = error;
            }
        }
    } 
    else if (target_runner_dist == 0) {
        return;
    }

    // Orbit state machine
    switch(orbit_state) {
        case ORBIT_NORMAL:
            orbit_normal();
            break;
        case ORBIT_TOOCLOSE:
            orbit_tooclose();
            break;
    }

    if((orbit_switch_ct > 6) && 
       (num_robots == 2)){
        set_motion(STOP);
        set_color(RGB(1,0,0));
        delay(100);
        set_color(RGB(0,0,0));
        delay(100);
        set_color(RGB(1,0,0));
        delay(100);
        stop();
    }
    
}

void runner_message_rx(message_t *m, distance_measurement_t *d){
    rx_kilo_id = m->data[0];
    new_message = 1;
    dist = *d;
}
message_t *runner_message_tx(){
    return &msg;
}

void runner_message_tx_success(){

}