#include <kilolib.h>
#include "runner.h"
#include "stationary.h"

typedef enum {
    RUNNER,
    STATIONARY
} role_t;

role_t current_role;
uint8_t num_robots = 4;
uint8_t current_runner = 0;
uint8_t runner_rx;

void setup() {
    if(kilo_uid == 0)
        current_role = RUNNER;
    else
        current_role = STATIONARY;
    
    runner_setup();
    stationary_setup();
}

void loop() {
    if(current_role == RUNNER){
        runner_loop();
    } else{
        stationary_loop();
    }
}

void message_rx(message_t *m, distance_measurement_t *d) {
    // Every bot's message is two parts: [0] is their own kilo_id and [1] is the kilo_id of what they think is the current runner
    // Only the current runner can change who the current runner is. Everyone else just rebroadcasts what the current runner is
    runner_rx = m->data[1];

    // If runner in message just received is not what the given bot thinks it is, then the current runner initialized a switch of runner
    // Update current runner, then initialize role switch on given bot. This includes running the respective setup.
    if (runner_rx != current_runner){
        current_runner = runner_rx;
        if (kilo_uid == current_runner){
            current_role = RUNNER;
            runner_setup();
            set_color(RGB(0,1,0));
            delay(20);
            set_color(RGB(0,0,0));
        } else {
            stationary_setup();
            current_role = STATIONARY;
        }
    }

    // Continuously check and update role
    if ((kilo_uid == current_runner) && (current_role != RUNNER)){
        runner_setup();
        current_role = RUNNER;
    } else if ((kilo_uid != current_runner) && (current_role != STATIONARY)) {
        stationary_setup();
        current_role = STATIONARY;
    }

    if(current_role == RUNNER)
        runner_message_rx(m, d);
    else
        stationary_message_rx(m, d);
}

message_t *message_tx(){
    if(current_role == RUNNER)
        return runner_message_tx();
    else
        return stationary_message_tx();
}

void message_tx_success(){
    if(current_role == RUNNER)
        runner_message_tx_success();
    else
        stationary_message_tx_success();
}

int main() {
    kilo_init();
    kilo_message_rx = message_rx;
    kilo_message_tx = message_tx;
    kilo_message_tx_success = message_tx_success;
    kilo_start(setup, loop);

    return 0;
}
