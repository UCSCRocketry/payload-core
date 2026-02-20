/**
 ******************************************************************************
 * @file           : log.h
 * @author         : Kyle Chen
 * @brief          : Simple UART logging (ERR, INF, WRN, DBG).
 ******************************************************************************
 */

#ifndef __LOG_H
#define __LOG_H

#include "stm32f4xx_hal.h"

	/** Call once after UART init to set the log output port. */
	void log_init(UART_HandleTypeDef *huart);

	/** Internal; used by LOG_* macros. */
	void log_msg(const char *level, const char *fmt, ...);
	void log_msg_raw(const char *fmt, ...);

#ifndef LOG_NODEBUG
#define LOG_DBG(fmt, ...) log_msg("DBG", fmt "\r\n", ##__VA_ARGS__)
#else
#define LOG_DBG(fmt, ...) ((void) 0)
#endif
#define LOG_ERR(fmt, ...) log_msg("ERR", fmt "\r\n", ##__VA_ARGS__)
#define LOG_INF(fmt, ...) log_msg("INF", fmt "\r\n", ##__VA_ARGS__)
#define LOG_WRN(fmt, ...) log_msg("WRN", fmt "\r\n", ##__VA_ARGS__)
#define LOG_RAW(fmt, ...) log_msg_raw(fmt "\r\n", ##__VA_ARGS__)

#endif /* __LOG_H */
