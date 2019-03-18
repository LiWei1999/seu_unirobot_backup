#ifndef UDP_COMM_DATA_H
#define UDP_COMM_DATA_H

#include "model.hpp"

#define COMM_DATA_HEADER "SEU"
struct comm_packet
{
    char header[3];
    unsigned int number=0;
    player_info info;
};


#endif