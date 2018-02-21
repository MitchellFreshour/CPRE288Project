/*
 * movement.c
 *
 *  Created on: Sep 8, 2016
 *      Author: wstheh
 */
#include "open_interface.h"
#include "movement.h"
#include "lcd.h"
#include "uart.h"

void movement_init() { ///init for moving the wheels
	oi_t *sensor_data = oi_alloc();
	oi_init(sensor_data);
}

/**
 * Turns the robot @param degrees in a clockwise direction
 * */
void turn_clockwise(oi_t *sensor_data, int degrees) {
	int sum = 0;
	oi_setWheels(-100, 100); /// right wheel max forward, left wheel max backward (to turn left)
	while (sum < degrees) {
		oi_update(sensor_data);
		sum += abs(sensor_data->angle);
	}
	oi_setWheels(0, 0); /// stop
}

/**
 * Turns the robot @param degrees in an anti-clockwise direction
 * */
void turn_anticlockwise(oi_t *sensor_data, int degrees) {
	int sum = 0;
	oi_setWheels(100, -100); /// right wheel max forward, left wheel max backward (to turn left)
	while (sum < degrees) {
		oi_update(sensor_data);
		sum += abs(sensor_data->angle);
	}
	oi_setWheels(0, 0); /// stop
}

/**
 * Moves the robot forward @param millimeters unless it hits a rock, a cliff, or reaches an edge
 * */
void move_forward(oi_t *sensor_data, int millimeters) {
	int sum = 0;
	lcd_init();
	oi_update(sensor_data);
	oi_setWheels(150, 125); /// move forward

	while ((sum < millimeters)) {
		oi_update(sensor_data);
		///if bump or cliff sensors are triggered
		if (((sensor_data->bumpLeft)) || ((sensor_data->bumpRight))
				|| (sensor_data->cliffLeft) || (sensor_data->cliffRight)
				|| (sensor_data->cliffFrontLeft)
				|| (sensor_data->cliffFrontRight)) {
            ///runs obstacle avoidance method
			obstacle1(sensor_data, 50);
			oi_update(sensor_data);

			sum = millimeters; ///sets so that the loop will end
		///if the light sensors run over the black circle used to denote the finishing spot
		} else if ((sensor_data->cliffFrontRightSignal < 1400)
				|| (sensor_data->cliffFrontLeftSignal < 1400)
				|| (sensor_data->cliffLeftSignal < 1400)
				|| (sensor_data->cliffRightSignal < 1400)) {
            ///runs end movement method
			end_move(sensor_data);
			oi_update(sensor_data);
			sum = millimeters; ///sets so that the loop will end
		}
		/// clif sensors signals for the white TAPE ////
		else if ((sensor_data->cliffFrontRightSignal > 2600)
				|| (sensor_data->cliffFrontLeftSignal > 2600)
				|| (sensor_data->cliffLeftSignal > 2600)
				|| (sensor_data->cliffRightSignal > 2600)) {
			///runs edge avoidance method
			edge_avoid(sensor_data);
			oi_update(sensor_data);
			sum = millimeters; ///sets so that the loop will end
		}
		oi_update(sensor_data);
		sum += abs(sensor_data->distance);
	}
	oi_setWheels(0, 0); /// stop
}

/**
 * Moves the robot backwards @param millimeters
 * */
void move_backwards(oi_t *sensor_data, int millimeters) {
	int sum = 0;
	oi_setWheels(-150, -150); /// similar to move forward accept the wheels are set negatively so they will turn in the opposite direction
	while (sum < millimeters) {

		oi_update(sensor_data);
		sum += abs(sensor_data->distance);

	}
	oi_setWheels(0, 0); // stop
}

/**
 * This is the avoidance proticol for the move_forward function, which is triggered
 * if the robot detects a cliff or a bump
 * */
void obstacle1(oi_t *sensor_data, int total_millimeters) { ///avoidance proticol

	int total_sum = 0;
    ///runs through all the sensors to determine which one was triggered, and then move backwards accordingly
	if (sensor_data->bumpLeft) { ///left bump
		oi_setWheels(0, 0);
		oi_update(sensor_data);
		move_backwards(sensor_data, total_millimeters);
		char BumpSENSOR[100];
		sprintf(BumpSENSOR, "Left Bump Sensor Activated \r\n");
		uart_sendStr(BumpSENSOR);

		//	turn_clockwise(sensor_data, 65);
		//   move_forward(sensor_data, 250);
		///  turn_anticlockwise(sensor_data, 65);

	} else if (sensor_data->bumpRight) { ///right bump
		oi_setWheels(0, 0);
		oi_update(sensor_data);
		move_backwards(sensor_data, total_millimeters);

		char BumpSENSOR[100];
		sprintf(BumpSENSOR, "Right Bump Sensor Activated \n\r");
		uart_sendStr(BumpSENSOR);

		//	turn_anticlockwise(sensor_data, 65);
		//	move_forward(sensor_data, 250);
		//	turn_clockwise(sensor_data, 65);

	}

	else if (sensor_data->bumpLeft && sensor_data->bumpRight) { ///left and right (object is in the center of the bot)
		oi_setWheels(0, 0);
		oi_update(sensor_data);
		move_backwards(sensor_data, total_millimeters);

		char BumpSENSOR[100];
		sprintf(BumpSENSOR,
				"Object in Center Bump Sensor R & L Activated \n\r");
		uart_sendStr(BumpSENSOR);
		/*
		 turn_clockwise(sensor_data, 65);
		 move_forward(sensor_data, 250);
		 turn_anticlockwise(sensor_data, 65); */

	}

	else if (sensor_data->cliffLeft) { ///cliff to the left
		oi_setWheels(0, 0);
		oi_update(sensor_data);
		move_backwards(sensor_data, total_millimeters);
		char CliffSENSOR[100];
		sprintf(CliffSENSOR, "left cliff sensor activated \n\r");
		uart_sendStr(CliffSENSOR);
	}

	else if (sensor_data->cliffFrontLeft) { ///cliff to the front left
		oi_setWheels(0, 0);
		oi_update(sensor_data);
		move_backwards(sensor_data, total_millimeters);
		char CliffSENSOR[100];
		sprintf(CliffSENSOR, "front left cliff sensor activated \n\r");
		uart_sendStr(CliffSENSOR);
	}

	else if (sensor_data->cliffFrontRight) { ///cliff to the front right
		oi_setWheels(0, 0);
		oi_update(sensor_data);
		move_backwards(sensor_data, total_millimeters);
		char CliffSENSOR[100];
		sprintf(CliffSENSOR, "front right cliff sensor activated \n\r");
		uart_sendStr(CliffSENSOR);
	}

	else if (sensor_data->cliffRight) { ///cliff to the right
		oi_setWheels(0, 0);
		oi_update(sensor_data);
		move_backwards(sensor_data, total_millimeters);
		char CliffSENSOR[100];
		sprintf(CliffSENSOR, "right cliff sensor activated \r\n");
		uart_sendStr(CliffSENSOR);
	}
	oi_update(sensor_data);
	total_sum += abs(sensor_data->distance);

	//     oi_update(sensor_data);
	//    sum += abs(sensor_data->distance);
	oi_setWheels(0, 0); // stop
}

/**
 * Movement proticol for when we reach the end dot.
 * */
void end_move(oi_t *sensor_data) {

	///if we hit the edge straight on
	if ((sensor_data->cliffFrontRightSignal < 1400)
			&& (sensor_data->cliffFrontLeftSignal < 1400)) {
		char EDGESENSOR[100];
		sprintf(EDGESENSOR, "BLACK DOT LEFT AND RIGHT \n\r");
		uart_sendStr(EDGESENSOR);
		///moves forward 10 cm
		int sum = 0;
		oi_setWheels(150, 150);
		while (sum < 10) {
			oi_update(sensor_data);
			sum += abs(sensor_data->distance);
		}
		oi_setWheels(0, 0);
	}

	///if left edge sensor is triggered
	else if ((sensor_data->cliffLeftSignal < 1400)
			|| (sensor_data->cliffFrontLeftSignal < 1400)) {
		char EDGESENSOR[100];
		sprintf(EDGESENSOR, "BLACK DOT LEFT EDGE ACTIVATED \n\r");
		uart_sendStr(EDGESENSOR);
		///turns 30 degrees to the left and moves forward 10 cm to put us more on the black dot
		int sum = 0;
		turn_anticlockwise(sensor_data, 30);
		oi_setWheels(150, 150);
		while (sum < 20) {
			oi_update(sensor_data);
			sum += abs(sensor_data->distance);
		}
		oi_setWheels(0, 0);
	}

	//if right edge sensor is activated
	else if ((sensor_data->cliffRightSignal < 1400)
			|| (sensor_data->cliffFrontRightSignal < 1400)) {
		char EDGESENSOR[100];
		sprintf(EDGESENSOR, "BLACK DOT RIGHT EDGE \n\r");
		uart_sendStr(EDGESENSOR);
		///turns 30 degrees to the right and moves forward 10 cm to put us more on the black dot
		int sum = 0;
		turn_clockwise(sensor_data, 30);
		oi_setWheels(150, 150);
		while (sum < 20) {
			oi_update(sensor_data);
			sum += abs(sensor_data->distance);
		}
		oi_setWheels(0, 0);
		//while still activated;

	}
}

/**
 * Avoidance proticol if we reach a white edge line. 
 * */
void edge_avoid(oi_t *sensor_data) {
	oi_update(sensor_data);
	oi_setWheels(0, 0);

//if we hit the edge straight on
	if ((sensor_data->cliffFrontRightSignal > 2600)
			&& (sensor_data->cliffFrontLeftSignal > 2600)) {
		char EDGESENSOR[100];
		sprintf(EDGESENSOR, "left & right edge sensor activated \n\r");
		uart_sendStr(EDGESENSOR);
		turn_anticlockwise(sensor_data, 200); ///simply turns 180 degrees

	}

//if left edge sensor is triggered
	else if ((sensor_data->cliffLeftSignal > 2600)
			|| (sensor_data->cliffFrontLeftSignal > 2600)) {
		int sum = 0;
		char EDGESENSOR[100];
		sprintf(EDGESENSOR, "left edge sensor activated \n\r");
		uart_sendStr(EDGESENSOR);

		///while the sensor is still activated
		oi_setWheels(100, -100); /// right wheel max forward, left wheel max backward (to turn left)
		while (((sensor_data->cliffLeftSignal > 2600)
				|| (sensor_data->cliffFrontLeftSignal > 2600))
				&& !(sensor_data->cliffFrontRightSignal > 2600)) {
			oi_update(sensor_data);
			if (!(sensor_data->cliffFrontLeftSignal > 2600
					|| sensor_data->cliffFrontRightSignal > 2600
					|| sensor_data->cliffLeftSignal > 2600)) {
				oi_setWheels(0, 0);
				oi_setWheels(100, 100);
				while (!(sensor_data->cliffFrontLeftSignal > 2600
						|| sensor_data->cliffFrontRightSignal > 2600
						|| sensor_data->cliffLeftSignal > 2600)) {
					oi_update(sensor_data);
				}
				oi_setWheels(0, 0);
				oi_setWheels(100, -100); /// right wheel max forward, left wheel max backward (to turn left)
			}

			sum += abs(sensor_data->angle);
		}
		oi_setWheels(0, 0); // stop
		turn_anticlockwise(sensor_data, 213);

	}
//if right edge sensor is activated
	else if ((sensor_data->cliffRightSignal > 2600)
			|| (sensor_data->cliffFrontRightSignal > 2600)) {
		int sum = 0;
		char EDGESENSOR[100];
		sprintf(EDGESENSOR, "right edge sensor activated \n\r");
		uart_sendStr(EDGESENSOR);

		//while still activated;

		oi_setWheels(-100, 100); /// right wheel max forward, left wheel max backward (to turn left)
		while (!(sensor_data->cliffFrontRightSignal > 2600
				&& sensor_data->cliffFrontLeftSignal > 2600)) {
			oi_update(sensor_data);
			if (!(sensor_data->cliffFrontRightSignal > 2600
					|| sensor_data->cliffFrontLeftSignal > 2600
					|| sensor_data->cliffRightSignal > 2600)) {
				oi_setWheels(0, 0);
				oi_setWheels(100, 100);
				while (!(sensor_data->cliffFrontRightSignal > 2600
						|| sensor_data->cliffFrontLeftSignal > 2600
						|| sensor_data->cliffRightSignal > 2600)) {
					oi_update(sensor_data);
				}
				oi_setWheels(0, 0);
				oi_setWheels(-100, 100); /// right wheel max forward, left wheel max backward (to turn left)
			}
			sum += abs(sensor_data->angle);
		}
		oi_setWheels(0, 0); // stop
		turn_clockwise(sensor_data, 220);

	}
}
