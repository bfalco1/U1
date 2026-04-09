#include <kilolib.h>

void stationary_message_rx(message_t *m, distance_measurement_t *d);
message_t *stationary_message_tx();
void stationary_message_tx_success();

void stationary_loop();
void stationary_setup();