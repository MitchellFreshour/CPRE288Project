/*
 * main.c
 */

#include "stdio.h"
#include "lcd.h"
#include "movement.h"
#include "timer.h"
#include <stdbool.h>
#include "driverlib/interrupt.h"
#include "button.h"
#include "math.h"
#include "WiFi.h"

//For PING sensor--------------------------------------------------------
#define TIMER3B_PRESCALER 0
volatile int state = 0, time1, time2, delta = 0;
//-----------------------------------------------------------------------

//For MAP plotting--------------------------------------------------------
#define mapScale 5
#define maxDistance 80
#define botSize 33/mapScale
#define xMap maxDistance/mapScale
#define yMap maxDistance/mapScale*2 // MAP[xMap][yMap];

int MAP[xMap][yMap] = { 0 };

//-----------------------------------------------------------------------

//for servo
unsigned pulse_period = (16000000 * 0.02); 	 //top or period
int count = 0;
unsigned int obj_count = 0;					//count objects
unsigned int flag_IR_low = 0;				//degree
unsigned int number_objects;

struct object {
	int objID;
	int distance1;
	int distance2;
	int min_distance;
	int degree1;
	int degree2;
	int width;
};

void move_servo(unsigned degree) {

	//0   -> 16M/0.001  = 8000
	//90  -> 24000
	//180 -> 16M/0.02 = 35200
	// 35200=m*180+8000
	//27200/180=m
	count = degree * ((27200) / 180) + 8000;	//1778;

	TIMER1_TBMATCHR_R = (pulse_period - count) & 0xFFFF;
	TIMER1_TBPMR_R = (pulse_period - count) >> 16;
	timer_waitMillis(100);
}

void timer1_init(void) {
	SYSCTL_RCGCGPIO_R |= 0x02; //start clock for Port B

	// Enable Digital functionality of Port B pin5
	GPIO_PORTB_DEN_R |= 0x20;

	//Enable alt function
	GPIO_PORTB_AFSEL_R |= 0x20;

	//Enable port control
	GPIO_PORTB_PCTL_R |= 0x00700000;

	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1; // Turn on clock to TIMER1

	//Configure the timer for PWM mode
	TIMER1_CTL_R &= ~(TIMER_CTL_TBEN); //disable timerB to allow us to change settings
	TIMER1_CFG_R |= TIMER_CFG_16_BIT; //set to 16 bit timer

	TIMER1_TBMR_R = (TIMER1_TBMR_R | (TIMER_TBMR_TBAMS) & ~TIMER_TBMR_TBCMR
			| TIMER_TBMR_TBMR_PERIOD);

	TIMER1_TBILR_R = pulse_period & 0xFFFF; //lower 16 bit of the interval
	TIMER1_TBPR_R = pulse_period >> 16;

	TIMER1_CTL_R |= TIMER_CTL_TBEN; //enable timer

}

//PING sensor--------------------------------------------------------------
void TIMER3B_Handler(void) {
	TIMER3_ICR_R = TIMER_ICR_CBECINT; //clear flag

	//read high
	if (state == 0) {
		time1 = TIMER3_TBV_R;
		state = 1;
	}

	//read low
	else if (state == 1) {
		time2 = TIMER3_TBV_R;
		state = 2;
		TIMER3_CTL_R &= ~(TIMER_CTL_TBEN);
	}

}

void clock_timer_init(void) {
	SYSCTL_RCGCGPIO_R |= 0x02; //start clock for Port B

	// Enable Digital functionality of Port B
	GPIO_PORTB_DEN_R |= 0x08;

	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R3; // Turn on clock to TIMER3

	//Configure the timer for input capture mode
	TIMER3_CTL_R &= ~(TIMER_CTL_TBEN); //disable timerB to allow us to change settings
	TIMER3_CFG_R |= TIMER_CFG_16_BIT; //set to 16 bit timer

	TIMER3_TBMR_R |=
			(TIMER_TBMR_TBCMR | TIMER_TBMR_TBMR_CAP | TIMER_TBMR_TBCDIR); //regular capture
	TIMER3_CTL_R |= TIMER_CTL_TBEVENT_BOTH; //rising and falling edge

	//TIMER3_TBPR_R = TIMER3B_PRESCALER - 1;  // 200 ms clock

	//TIMER3_TBILR_R = highest possible value by default;   //(int)(16000000/(TIMER3B_PRESCALER * 5)); // set the load value for the 0.2 second clock

	//clear TIMER3B interrupt flags
	TIMER3_ICR_R |= TIMER_ICR_CBECINT; //clears TIMER3 time-out interrupt flags
	TIMER3_IMR_R |= TIMER_IMR_CBEIM; //enable TIMER3(A&B) time-out interrupts

	//initialize local interrupts
	NVIC_EN1_R = 0x00000010; //#warning "enable interrupts for TIMER3B" n = 0, 1, 2, 3, or 4
	//go to page 105 and find the corresponding interrupt numbers for TIME3B
	//then set those bits in the correct interrupt set EN register (p. 142)

	IntRegister(INT_TIMER3B, TIMER3B_Handler); //register TIMER3B interrupt handler

	IntMasterEnable(); //intialize global interrupts

}

void send_pulse() {

	//Disable alt function
	GPIO_PORTB_AFSEL_R &= ~(0x08);

	GPIO_PORTB_PCTL_R &= ~(0xF000);
	TIMER3_CTL_R &= ~(TIMER_CTL_TBEN);

	//Set direction of Port B, pin 3 output
	GPIO_PORTB_DIR_R |= 0x08;

	//Send high
	GPIO_PORTB_DATA_R |= 0x08;

	//wait 5us
	timer_waitMicros(6);

	//Send low
	GPIO_PORTB_DATA_R &= ~(0x08);

	//Set direction of Port B, pin 3 input
	GPIO_PORTB_DIR_R &= ~(0x08);

	//Enable alt function
	GPIO_PORTB_AFSEL_R |= 0x08;

	//Enable port control
	GPIO_PORTB_PCTL_R |= 0x00007000;

	TIMER3_CTL_R |= TIMER_CTL_TBEN; //Enable TIMER3B
}
//------------------------------------------------------------------------------------

//for IR -----------------------------------------------------------------------------
void adc_int() {
	//enable ADC 0 module on port D
	SYSCTL_RCGCGPIO_R |= 0b000010;
	//enable clock for ADC
	SYSCTL_RCGCADC_R |= 0x1;
	//enable port D pin 0 to work as alternate functions
	GPIO_PORTB_AFSEL_R |= 0x10;
	//set pin to input - 0
	GPIO_PORTB_DIR_R &= ~(0x10);
	//disable analog isolation for the pin
	GPIO_PORTB_AMSEL_R |= 0x01;
	//initialize the port trigger source as processor (default)
	GPIO_PORTB_ADCCTL_R = 0x00;

	//disable SS0 sample sequencer to configure it
	ADC0_ACTSS_R &= ~(ADC_ACTSS_ASEN0);	//what is sample sequencer and why disable it now then reenable later  ;;ADC_ACTSS_ASEN0
	//initialize the ADC trigger source as processor (default)
	ADC0_EMUX_R = ADC_EMUX_EM0_PROCESSOR;			//ADC_EMUX_EM0_PROCESSOR;
	//set 1st sample to use the AIN10 ADC pin
	ADC0_SSMUX0_R |= 0x000A;							//why bit 3 and 1? pg851
	//enable raw interrupt
	ADC0_SSCTL0_R |= (ADC_SSCTL0_IE0 | ADC_SSCTL0_END0);//pg853 interrupt enable and end of sequence
	//enable oversampling to average
	ADC0_SAC_R |= ADC_SAC_AVG_16X;							//ADC_SAC_AVG_64X;
	//re-enable ADC0 SS0
	ADC0_ACTSS_R |= ADC_ACTSS_ASEN0;

	//initiate SS1 conversion
	ADC0_PSSI_R = ADC_PSSI_SS0;	//initiate ss1 but why? arent we using ss2? ADC_PSSI_SS1
	//wait for ADC conversion to be complete
	while ((ADC0_RIS_R & 0x0001) == 0) {
		//wait
	}
	//grab result
	int value = ADC0_SSFIFO1_R;

	//initiate SS0 conversion
	ADC0_PSSI_R = ADC_PSSI_SS0;
	//wait for ADC conversion to be complete
	while ((ADC0_RIS_R & ADC_RIS_INR0) == 0) {
		//wait
	}
	//clear interrupt
	ADC0_ISC_R = ADC_ISC_IN0;

}

unsigned ADC_read(char channel) {

	//disable ADC0SS0 sample sequencer to configure it
	ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN0;
	//set 1st sample to use the channel ADC pin
	ADC0_SSMUX0_R |= channel;
	//re-enable ADC0 SS0
	ADC0_ACTSS_R |= ADC_ACTSS_ASEN0;
	//initiate SS0 conversion
	ADC0_PSSI_R = ADC_PSSI_SS0;
	//wait for ADC conversion to be complete
	while ((ADC0_RIS_R & ADC_RIS_INR0) == 0) {
	}
	//clear interrupt
	ADC0_ISC_R = ADC_ISC_IN0;

	return ADC0_SSFIFO0_R;
}
//--------------------------------------------------------------------------------------------
//PING sensor----------
int ping() {

	unsigned int distance;

	send_pulse();

	//ping_read();
	while (state != 2) {
	}
	delta = time2 - time1;
	distance = (int) delta * 0.001063;	//((1.0/16000000)*(.5)*(34000));
	state = 0;

	//if (distance > maxDistance){
	//	distance = maxDistance;
	//}

	return distance;
}

void empty_array() {

	int i, j;

	for (i = 0; i < xMap; i++) {
		for (j = 0; j < yMap; j++) {
			if (MAP[i][j] == 1) {
				MAP[i][j] = 0;
			}

		}
	}

}

void scan_function(struct object object[20]) {

	//From IR lab 6
	obj_count = 0;
	int value = 0;
	int distance_IR = 0;
	int i, j;
	unsigned int degree = 0;
	unsigned int avoid_spikes = 0;		//avoid spike
	unsigned int distance; 				//PING sensor
	int flag = 0;		// if sudden spike in middle of object
	int min_distance = 0;

	//variables used to count width and smallest object
	unsigned int min = 100, smallest_object;
	unsigned int a, b, C;

	char data[100] = "Degree,\t Distance_PING,\t Distance_IR,\n\r";
	uart_sendStr(data);
	move_servo(0);
	while (degree != 182) {

		move_servo(degree);

		//distance = ping();

		//IR sensor---------------
		value = ADC_read(0);
		distance_IR = 266864 * pow(value, -1.301);

		//	last night work

		if (distance_IR < 80 && flag_IR_low == 0) { //&& degree > 2){                   //start detecting object
			timer_waitMillis(100);
			distance = ping();
			if (distance < maxDistance) {
				flag_IR_low = 1;
				object[obj_count].objID = obj_count;
				min_distance = distance;
				object[obj_count].distance1 = distance;
				object[obj_count].degree1 = degree;
			}

		} else if (distance_IR < 80 && flag_IR_low == 1 && distance < maxDistance) { //update object degree2
			timer_waitMillis(100);
			distance = ping();
			if (distance < min_distance && distance != 0){
				min_distance = distance;
			}
			object[obj_count].distance2 = distance;
			object[obj_count].degree2 = degree;
			avoid_spikes++;
			flag = 0;

			if (degree == 180) {
				obj_count++;
			}

		} else if (distance_IR > 80 && flag_IR_low == 1 && avoid_spikes > 3 && flag == 0) { //object finished calculating
				//object[obj_count].width = sqrt(pow(object[obj_count].distance1, 2) + pow(object[obj_count].distance2, 2) - (2 * object[obj_count].distance1 * object[obj_count].distance2 * cos((object[obj_count].distance2 - object[obj_count].distance1) * 3.1415 / 180)) );
			flag = 1;

		} else if (distance_IR > 80 && flag_IR_low == 1 && avoid_spikes > 3 && flag == 1) { //object finished calculating

			flag_IR_low = 0;
			avoid_spikes = 0;
			object[obj_count].min_distance = min_distance;

			obj_count++;	//move to next object in array

		} else {

			flag_IR_low = 0;
			distance = 0;
			min_distance = 0;
		}

		//print out onto putty
		sprintf(data, "%d,\t %d,\t\t %d,\n\r", degree, distance, distance_IR);
		uart_sendStr(data);
		degree = degree + 2;

	}

	sprintf(data, "obj count= %d\n\r", obj_count);
	uart_sendStr(data);

	//calculate width and smallest object
	for (j = 0; j < obj_count; j++) {

		a = object[j].min_distance;
		C = (object[j].degree2 - object[j].degree1) / 2;		//in deg


		b = a * tan(C * 3.1415 / 180);
		object[j].width = b * 2;

		if (object[j].width < min) {
			min = object[j].width;		//smallest object width
			smallest_object = (object[j].degree1 + object[j].degree2) / 2;//smallest object degree
		}

		sprintf(data, "Object = %d, Degree = %d, Width = %d, Distance = %d\n\r",
				j, ((object[j].degree2 + object[j].degree1) / 2),
				object[j].width,
				object[j].min_distance);
		uart_sendStr(data);
	}

	lcd_printf("Object count = %d\n Degree = %d\n Widthsquare = %d", obj_count,
			smallest_object, min);

}

void drawMap(int *MAP, struct object object[20]) {
	int i, x, y;
	for (i = 0; i < obj_count; i++) {

		if (object[i].degree1 == 0) {
			x = object[i].distance1 / mapScale;
			MAP[(xMap - 1) * yMap + (yMap / 2) + x] = 1;
		} else if (object[i].degree1 < 90) {
			x = (object[i].distance1 * cos(object[i].degree1 * 3.1415 / 180))
					/ mapScale;
			y = (object[i].distance1 * sin(object[i].degree1 * 3.1415 / 180))
					/ mapScale;
			MAP[(xMap - y) * yMap + (yMap / 2) + x] = 1;
		} else if (object[i].degree1 > 90) {
			x = (object[i].distance1
					* cos((180 - object[i].degree1) * 3.1415 / 180)) / mapScale;
			y = (object[i].distance1
					* sin((180 - object[i].degree1) * 3.1415 / 180)) / mapScale;
			MAP[(xMap - y) * yMap + (yMap / 2) - x] = 1;
		}

		if (object[i].degree2 == 180) {
			x = object[i].distance2 / mapScale;
			MAP[(xMap - 1) * yMap + (yMap / 2) - x] = 1;
		} else if (object[i].degree2 < 90) {
			x = (object[i].distance2 * cos(object[i].degree2 * 3.1415 / 180))
					/ mapScale;
			y = (object[i].distance2 * sin(object[i].degree2 * 3.1415 / 180))
					/ mapScale;
			MAP[(xMap - y) * yMap + (yMap / 2) + x] = 1;
		} else if (object[i].degree2 > 90) {
			x = (object[i].distance2
					* cos((180 - object[i].degree2) * 3.1415 / 180)) / mapScale;
			y = (object[i].distance2
					* sin((180 - object[i].degree2) * 3.1415 / 180)) / mapScale;
			MAP[(xMap - y) * yMap + (yMap / 2) - x] = 1;
		}

	}
}

void printMAP() {

	int i, j;
	for (i = 0; i < xMap; i++) {
		uart_sendChar('|');
		//uart_sendChar(' ');
		for (j = 0; j < yMap; j++) {
			if (MAP[i][j] == 1) {
				uart_sendChar('o');
				//uart_sendChar(' ');
			} else if (i == xMap - 1
					&& (j > ((yMap / 2) - botSize / 2)
							&& j < ((yMap / 2) + botSize / 2))) {
				uart_sendChar('x');
				//uart_sendChar(' ');

			} else if (i == xMap - 1
					&& (j < ((yMap / 2) - botSize / 2)
							&& j > ((yMap / 2) + botSize / 2))) {
				uart_sendChar('_');
				//uart_sendChar(' ');

			} else if (MAP[i][j] == 0) {
				uart_sendChar(' ');
				//uart_sendChar(' ');
			}
		}
		uart_sendChar('|');
		uart_sendChar('\r');
		uart_sendChar('\n');
	}

}

void flushWait();

void load_songs() {

	unsigned char mario1NumNotes = 49;
	unsigned char mario1Notes[49] = { 62, 0, 62, 0, 62, 0, 65, 67, 0,  62, 0, 62, 0, 62, 0, 60, 61, 0,
			62, 0, 62, 0, 62, 0, 65, 67, 0,  62, 0, 62, 0, 62, 0, 60, 61, 0, 77, 74, 69, 0, 77,
			74, 68, 0, 67, 68};

	unsigned char mario1Duration[49] = { 12, 5, 12, 5, 12, 5, 12, 12, 12, 12, 5, 12, 5, 12, 5, 12, 12, 12,
			12, 5, 12, 5, 12, 5, 12, 12, 12, 12, 5, 12, 5, 12, 5, 12, 12, 12, 6, 6, 30, 36, 6,
			6, 30, 36, 18, 24 };

	oi_loadSong(0, mario1NumNotes, mario1Notes, mario1Duration);

}

int main(void) {
    
    //Initlaizing everything we need for our program
	lcd_init();
	uart_init();
	clock_timer_init();
	timer1_init();
	adc_int();
	move_servo(0);
	timer_waitMillis(500);

	struct object object[20]; //structure for the objects found during a scan

    //WiFi_start("Kalamazoo"); 

	oi_t *sensor_data = oi_alloc();

	oi_init(sensor_data);
	load_songs();
	char message[100];
	int j = 0;

	//uart_flush();
	while (1) {
		//	lcd_printf("Value left : %d,\nright: %d", sensor_data->cliffFrontLeftSignal, sensor_data->cliffFrontRightSignal);
		char data = uart_receive(); //recieves command from putty
		flushWait();
		timer_waitMillis(100);
		//logic statements to use the command given from putty to preform a designated action
		if (data == 'w') {//moves forward 20 cm
			move_forward(sensor_data, 85);
			sprintf(message, "Moved forward 20cm\n\r", j);
			uart_sendStr(message);
		} else if (data == 's') { //moves forward 10 cm

			move_forward(sensor_data, 42);
			sprintf(message, "Moved forward 10cm\n\r", j);
			uart_sendStr(message);
		} else if (data == 'x') { //moves forward 5 cm
			move_forward(sensor_data, 21);
			sprintf(message, "Moved forward 5cm\n\r", j);
			uart_sendStr(message);
		} else if (data == 'q') { //moves backward 20 cm
			move_backwards(sensor_data, 200);
			sprintf(message, "Moved backward 20cm\n\r", j);
			uart_sendStr(message);
		} else if (data == 'a') { //moves backward 10 cm
			move_backwards(sensor_data, 100);
			sprintf(message, "Moved backward 10cm\n\r", j);
			uart_sendStr(message);
		} else if (data == 'z') { //moves backward 5cm
			move_backwards(sensor_data, 50);
			sprintf(message, "Moved backward 5cm\n\r", j);
			uart_sendStr(message);
		} else if (data == 'e') { //turns the robot left 90 degrees
			turn_anticlockwise(sensor_data, 102);
			sprintf(message, "Turned left\n\r", j);
			uart_sendStr(message);
		} else if (data == 'r') { //turns the robot right 90 degrees
			turn_clockwise(sensor_data, 102);
			sprintf(message, "Turned right\n\r", j);
			uart_sendStr(message);
		} else if (data == 'm') { //scans for objects using the IR and Sonar sensors
			empty_array();
			scan_function(object);
			drawMap(&MAP[0][0], object);
			printMAP();

			for (j = 0; j < obj_count; j++) { //alerts us to objects that may be a goal post
				if (object[j].width < 6) {
					sprintf(message, "   Object %d is a goal post!\n\r", j);
					uart_sendStr(message);
				}
			}
		} else if (data == 'c') { //plays a song loaded onto the robot
			oi_play_song(0);
		}

		else if (data == 'E') { //turns the robot left 45 degrees
			turn_anticlockwise(sensor_data, 52);
			sprintf(message, "Turned Left 45\n\r", j);
			uart_sendStr(message);
		} else if (data == 'R') { //turns the robot right 45 degrees
			turn_clockwise(sensor_data, 51);
			sprintf(message, "Turned Right 45 \n\r", j);
			uart_sendStr(message);
		}
        //small angle adjustments
		else if (data == 'f') { //turns left 5 degrees
			turn_anticlockwise(sensor_data, 6);
			sprintf(message, "Turned Left 5 \n\r", j);
			uart_sendStr(message);
		} else if (data == 'F') { //turns right 5 degrees
			turn_clockwise(sensor_data, 6);
			sprintf(message, "Turned Right 5 \n\r", j);
			uart_sendStr(message);
		}

		//lcd_clear();
		uart_sendChar('\r');
		//lcd_putc(data);
	}

}

void flushWait() {
	int i = 0;     //to flush the data quick
	while (i++ < 10000)   //it wait about 10th of a second
	{
		uart_flush();
		timer_waitMicros(10);   //wait and flush it in 10micro
	}
}

