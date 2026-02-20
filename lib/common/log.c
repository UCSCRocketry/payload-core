/**
 ******************************************************************************
 * @file           : log.c
 * @author         : Kyle Chen
 * @brief          : Simple UART logging.
 ******************************************************************************
 */

#include "log.h"
#include <stdarg.h>
#include <stdio.h>

static UART_HandleTypeDef *s_huart;

void log_init(UART_HandleTypeDef *huart)
{
	s_huart = huart;
}

void log_msg(const char *level, const char *fmt, ...)
{
	if (!s_huart)
	{
		return;
	}
	static char buf[192];
	int n = snprintf(buf, sizeof(buf), "[%s] ", level);
	if (n < 0 || (size_t) n >= sizeof(buf))
	{
		return;
	}
	va_list args;
	va_start(args, fmt);
	n += vsnprintf(buf + n, sizeof(buf) - (size_t) n, fmt, args);
	va_end(args);
	if (n < 0 || (size_t) n >= sizeof(buf))
	{
		n = (int) (sizeof(buf) - 1);
	}
	HAL_UART_Transmit(s_huart, (uint8_t *) buf, (uint16_t) (size_t) n, 100);
}

void log_msg_raw(const char *fmt, ...)
{
	if (!s_huart)
	{
		return;
	}
	static char buf[192];
	va_list args;
	va_start(args, fmt);
	int n = vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	if (n <= 0 || (size_t) n > sizeof(buf))
	{
		return;
	}
	HAL_UART_Transmit(s_huart, (uint8_t *) buf, (uint16_t) (size_t) n, 100);
}
