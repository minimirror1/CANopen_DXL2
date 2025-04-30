/*
 * tick.cpp
 *
 *  Created on: Jun 15, 2023
 *      Author: minim
 */

#include "cpp_tick.h"

Tick::Tick(){
	tickTime = getTickCount();
}

Tick::~Tick(){

}


bool Tick::delay(uint32_t time){
	if(elapsed(tickTime) >= time) {
		tickTime = HAL_GetTick();
		return true;
	}
	else {
		return false;
	}
}

uint32_t Tick::elapsed(uint32_t time){
    if (HAL_GetTick() >= time) 	return (HAL_GetTick() - time               );
    else						return ((0xffffffff - time)+ HAL_GetTick()+1);
}

uint32_t Tick::getElapsed(uint32_t time1, uint32_t time2){
    if (time1 >= time2) return (time1 - time2);
    else                return ((0xffffffff-time2) + time1+1);
}


void Tick::tickUpdate(){
	tickTime = getTickCount();
}
bool Tick::tickCheck(uint32_t time){
	if(elapsed(tickTime) >= time)
		return true;
	else
		return false;
}
