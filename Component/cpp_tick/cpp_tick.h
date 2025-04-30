/*
 * tick.h
 *
 *  Created on: Jun 15, 2023
 *      Author: minim
 */

#ifndef INC_TICK_H_
#define INC_TICK_H_


#include "main.h"

class Tick{

public:
	Tick();
	~Tick();

	bool delay(uint32_t time);

	uint32_t elapsed(uint32_t time);
	uint32_t getElapsed(uint32_t time1, uint32_t time2);
	uint32_t getTickCount(void){ return HAL_GetTick(); }

	void tickUpdate();
	bool tickCheck(uint32_t time);

private:
	uint32_t tickTime;
};



#endif /* INC_TICK_H_ */
