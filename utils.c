#include "utils.h"

uint8_t id_to_color(uint16_t id, uint8_t num_robots){
    if(num_robots == 3){
        if(id == 0){
            return RGB(1, 0, 0);
        }
        else if(id == 1){
            return RGB(0, 1, 0);
        }
        else if(id == 2){
            return RGB(0, 0, 1);
        }
    }
    else if(num_robots == 4){
        if(id == 0){
            return RGB(1, 0, 0);
        }
        else if(id == 1){
            return RGB(0, 1, 0);
        }
        else if(id == 2){
            return RGB(0, 0, 1);
        }
        else if(id == 3){
            return RGB(0, 1, 1);
        }
    }
    return 0;
}