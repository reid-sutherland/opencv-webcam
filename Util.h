/*
 * Util.h
 *
 *  Created on: 19 sept. 2015
 *      Author: erman
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <stdio.h>
#include <stdarg.h>
#include <string>
#include <ctime>

using namespace std;

class Util {


public:
	Util();
	virtual ~Util();

	static string stringf(const char* format, ...);
	static const string currentDateTime();
	static const string currentDateTime(string);
};

#endif /* UTIL_H_ */
