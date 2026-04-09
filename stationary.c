#include "stationary.h"
#include "utils.h"

// Take current_runner from main
extern uint8_t current_runner;
extern uint8_t num_robots;



uint8_t front_kilo_id; // id of the bot in the front
uint8_t second_kilo_id; // id of bot behind front one; bot used to know when to stop runner at the front
uint8_t rx_kilo_id; // id of bot that sent message
uint8_t curr_dist = UINT8_MAX;
uint8_t runner_dist = UINT8_MAX;

uint8_t message_sent;
message_t msg_tx;
message_t msg_rx;
uint8_t new_messsage = 0;
distance_measurement_t dist;

void stationary_setup(){
    message_sent = 0;
    msg_tx.type = NORMAL;
    msg_tx.data[0] = kilo_uid;
    msg_tx.data[1] = current_runner;
    msg_tx.data[2] = UINT8_MAX;
    msg_tx.crc = message_crc(&msg_tx);

    front_kilo_id = (current_runner - 1 + num_robots) % num_robots;
    second_kilo_id = (front_kilo_id - 1 + num_robots) % num_robots;
    set_color(id_to_color(kilo_uid, num_robots));
    set_motors(0,0);
}

uint32_t last_ticks = 0;
void stationary_loop(){
    if(new_messsage){
        new_messsage = 0;

        // If first stationary kilobot, report distance to second stationary kilobot
        if((kilo_uid == front_kilo_id) && (msg_rx.data[0] == second_kilo_id)){
            msg_tx.data[2] = estimate_distance(&dist);
            msg_tx.crc = message_crc(&msg_rx);
        }

        // Set color according to distance from runner
        // if(msg_rx.data[0] == current_runner){
        //     set_color(estimate_distance(&dist));
        // }
        set_color(id_to_color(kilo_uid, num_robots));
    }

    if((kilo_uid == front_kilo_id) && (kilo_ticks - last_ticks > 64)){
        last_ticks = kilo_ticks;
        set_color(0);
        delay(50);
        set_color(id_to_color(kilo_uid, num_robots));
    }

    // Continuously update current_runner in message
    if (num_robots != 2){
        msg_tx.data[1] = current_runner;
        msg_tx.crc = message_crc(&msg_tx);
    }


}

void stationary_message_rx(message_t *m, distance_measurement_t *d){
    msg_rx = *m;
    dist = *d;
    new_messsage = 1;
}

message_t *stationary_message_tx(){
    return &msg_tx;
}

void stationary_message_tx_success(){
    message_sent = 1;
}