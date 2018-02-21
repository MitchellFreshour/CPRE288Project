/*
 * movement.h
 *
 *  Created on: Sep 8, 2016
 *      Author: wstheh
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "Timer.h"
#include <inc/tm4c123gh6pm.h>
#include "open_interface.h"

void movement_init();

void turn_clockwise(oi_t *sensor_data, int degrees);

void turn_anticlockwise(oi_t *sensor_data, int degrees);

void move_forward(oi_t *sensor_data, int millimeters);

void move_backwards(oi_t *sensor_data, int millimeters);

void obstacle1(oi_t *sensor_data, int total_millimeters); //avoidance proticol

void end_move(oi_t *sensor_data);

void edge_avoid(oi_t *sensor_data);

#endif /* MOVEMENT_H_ */
