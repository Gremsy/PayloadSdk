#ifndef	__LOG_H
#define __LOG_H

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cstdarg>

#define ERR_LEVEL		0
#define WARN_LEVEL		1
#define INFO_LEVEL      2
#define DEBUG_LEVEL     3

#define LOG_LEVEL       DEBUG_LEVEL

void PRINT_ERR(const char* msg,...){
#if (LOG_LEVEL >= ERR_LEVEL)
	va_list args;
	va_start(args,msg);
	printf("\x1b[31m[Error] | ");
	vprintf(msg,args);
	printf("\x1b[0m\r\n");
	va_end(args);
#endif
}

void PRINT_WARN(const char* msg,...){
#if (LOG_LEVEL >= WARN_LEVEL)
	va_list args;
	va_start(args,msg);
	printf("\x1b[33m[Warn] | ");
	vprintf(msg,args);
	printf("\x1b[0m\r\n");
	va_end(args);
#endif
}

void PRINT_INFO(const char* msg,...){
#if (LOG_LEVEL >= INFO_LEVEL)
	va_list args;
	va_start(args,msg);
	printf("\x1b[32m[Info] | ");
	vprintf(msg,args);
	printf("\x1b[0m\r\n");
	va_end(args);
#endif
}

void PRINT_DEBUG(const char* msg,...){
#if (LOG_LEVEL >= DEBUG_LEVEL)
	va_list args;
	va_start(args,msg);
	printf("\x1b[0m[Debug] | ");
	vprintf(msg,args);
	printf("\x1b[0m\r\n");
	va_end(args);
#endif
}


#endif