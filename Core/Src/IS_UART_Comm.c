/*
 * IS_UART_Comm.c
 *
 *  Created on: Dec 26, 2023
 *      Author: fewletter
 */

#include "IS_UART_Comm.h"

void CommPacket_Read(UART_HandleTypeDef* huart, uint8_t* buffer, uint8_t size, CommPacket* comm)
{
	HAL_UART_Receive(huart, buffer, size, 12);
    strcpy(comm->buffer, buffer);
}

int IS_UART_BuildComm(CommPacket* comm)
{
	for (int i = 0; i < UART_BUFFER_SIZE; i++) {
		if (comm->buffer[i] == '$' && comm->buffer[i+1] == 'P' && comm->buffer[i+2] == 'I' && comm->buffer[i+3] == 'M' && comm->buffer[i+4] == 'U') {
			comm->head = comm->buffer + i;
			comm->tail = comm->head == comm->buffer ? (comm->buffer + UART_BUFFER_SIZE - 1) : (comm->head - 1);
			comm->datatype = PIMU;
			break;
		}
		else if (comm->buffer[i] == '$' && comm->buffer[i+1] == 'P' && comm->buffer[i+2] == 'I' && comm->buffer[i+3] == 'N' && comm->buffer[i+4] == 'S') {
			comm->head = comm->buffer + i;
			comm->tail = comm->head == comm->buffer ? (comm->buffer + UART_BUFFER_SIZE - 1) : (comm->head - 1);
			comm->datatype = PINS1;
			break;
		}
	}
	comm->breakpoint = comm->buffer + UART_BUFFER_SIZE - 1;
}

bool IS_UART_checkAsciidigit(uint8_t digit)
{
	return ((digit - '0') >= 0 && (digit - '9') <= 0) ? true : false;
}

int IS_UART_DatamapScooter(CommPacket* comm, E_Scooter* state)
{
	uint8_t comma_count = 0;
	bool sign = 0;
	uint8_t* start = 0;
	float newdelta = 0.0f;
	float newtheta = 0.0f;
	float newlinearv = 0.0f;
    for (uint8_t* ptr = comm->head; ptr != comm->tail; ptr++) {

    	if (*ptr == ',') {
			comma_count += 1;
			sign = *(ptr+1) == '-' ? 1 : 0;
			start = ptr + 1 + sign;

			if (comm->datatype == PIMU) {
				switch (comma_count) {
				case 2:
					newdelta = (IS_UART_checkAsciidigit(*start) ?  *start - '0': 0)
							+ (IS_UART_checkAsciidigit(*(start+2)) ?  (*(start+2) - '0') * 0.1: 0)
							+ (IS_UART_checkAsciidigit(*(start+3)) ?  (*(start+3) - '0') * 0.01: 0)
							+ (IS_UART_checkAsciidigit(*(start+4)) ?  (*(start+4) - '0') * 0.001: 0)
							+ (IS_UART_checkAsciidigit(*(start+5)) ?  (*(start+5) - '0') * 0.0001: 0);
					newdelta = sign ? newdelta * (-1)  : newdelta;
					break;
				case 4:
					newtheta = (IS_UART_checkAsciidigit(*start) ?  *start - '0': 0)
							+ (IS_UART_checkAsciidigit(*(start+2)) ?  (*(start+2) - '0') * 0.1: 0)
							+ (IS_UART_checkAsciidigit(*(start+3)) ?  (*(start+3) - '0') * 0.01: 0)
							+ (IS_UART_checkAsciidigit(*(start+4)) ?  (*(start+4) - '0') * 0.001: 0)
							+ (IS_UART_checkAsciidigit(*(start+5)) ?  (*(start+5) - '0') * 0.0001: 0);
					newtheta = sign ? newtheta * (-1)  : newtheta;
					break;
				case 7:
					newlinearv = (IS_UART_checkAsciidigit(*start) ?  *start - '0': 0)
							+ (IS_UART_checkAsciidigit(*(start+2)) ?  (*(start+2) - '0') * 0.1: 0)
							+ (IS_UART_checkAsciidigit(*(start+3)) ?  (*(start+3) - '0') * 0.01: 0)
							+ (IS_UART_checkAsciidigit(*(start+4)) ?  (*(start+4) - '0') * 0.001: 0)
							+ (IS_UART_checkAsciidigit(*(start+5)) ?  (*(start+5) - '0') * 0.0001: 0);
					newlinearv = sign ? newlinearv * (-1)  : newlinearv;
					break;
				}
			}
			else if (comm->datatype == PINS1) {
				switch (comma_count) {
				case 5:
					state->theta = (IS_UART_checkAsciidigit(*start) ?  *start - '0': 0)
							+ (IS_UART_checkAsciidigit(*(start+2)) ?  (*(start+2) - '0') * 0.1: 0)
							+ (IS_UART_checkAsciidigit(*(start+3)) ?  (*(start+3) - '0') * 0.01: 0)
							+ (IS_UART_checkAsciidigit(*(start+4)) ?  (*(start+4) - '0') * 0.001: 0)
							+ (IS_UART_checkAsciidigit(*(start+5)) ?  (*(start+5) - '0') * 0.0001: 0);
					state->theta = sign ? state->theta * (-1)  : state->theta;
					break;
				case 6:
					state->delta = (IS_UART_checkAsciidigit(*start) ?  *start - '0': 0)
							+ (IS_UART_checkAsciidigit(*(start+2)) ?  (*(start+2) - '0') * 0.1: 0)
							+ (IS_UART_checkAsciidigit(*(start+3)) ?  (*(start+3) - '0') * 0.01: 0)
							+ (IS_UART_checkAsciidigit(*(start+4)) ?  (*(start+4) - '0') * 0.001: 0)
							+ (IS_UART_checkAsciidigit(*(start+5)) ?  (*(start+5) - '0') * 0.0001: 0);
					state->delta = sign ? state->delta * (-1)  : state->delta;
					break;
				case 7:
					state->linearv = (IS_UART_checkAsciidigit(*start) ?  *start - '0': 0)
							+ (IS_UART_checkAsciidigit(*(start+2)) ?  (*(start+2) - '0') * 0.1: 0)
							+ (IS_UART_checkAsciidigit(*(start+3)) ?  (*(start+3) - '0') * 0.01: 0)
							+ (IS_UART_checkAsciidigit(*(start+4)) ?  (*(start+4) - '0') * 0.001: 0)
							+ (IS_UART_checkAsciidigit(*(start+5)) ?  (*(start+5) - '0') * 0.0001: 0);
					state->linearv = sign ? state->linearv * (-1)  : state->linearv;
					break;
				}
			}
    	}
    	if (ptr == comm->breakpoint)
    		ptr = comm->buffer - 1;

    }

    if (comm->datatype == PIMU) {
    	state->delta += newdelta;
		state->theta += newtheta;
		state->linearv += newlinearv;
    }

}

