/*
 * Task.h
 *
 *  Created on: Jan 18, 2018
 *      Author: ofir
 */

#ifndef TASK_H_
#define TASK_H_

#include <map>
#include <string>
#include "Command.h"

class Task {
public:
	Task(void * (*func)(void *arg), void* arg) :
			func(func), arg(arg) {
	}
	void execute() {
		func(arg);
	}
	virtual ~Task() {}

	void* getArg() const {
		return arg;
	}

private:
	void * (*func)(void *arg);
	void *arg;
};

#endif /* TASK_H_ */
