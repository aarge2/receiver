#ifndef EDGE_DETECTION_H
#define EDGE_DETECTION_H

#include "stdbool.h"
#include "stdint.h"

enum 
{
	ED_ID_0,
	ED_ID_1,
	ED_ID_END,
};

bool edge_detection(uint8_t b_id, bool o_catch);

#endif
