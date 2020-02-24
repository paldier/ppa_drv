/***************************************************************************
 *
 * Copyright (C) 2017-2018 Intel Corporation
 * Copyright (C) 2010-2016 Lantiq Beteiligungs-GmbH & Co. KG
 * Author  : Purnendu Ghosh
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * SPDX-License-Identifier: GPL-2.0
 *
 ***************************************************************************/

#ifndef _qos_hal_debug_h
#define _qos_hal_debug_h

#if defined(_DEBUG)
/** enable debug printouts */
#  define INCLUDE_DEBUG_SUPPORT
#endif
void qos_hal_proc_destroy(void);
int qos_hal_proc_create(void);

#define QOS_DEBUG_ERR (1<< 0)
#define QOS_DEBUG_TRACE (1<<1)
#define QOS_DEBUG_HIGH (1<<2)
#define QOS_ENABLE_ALL_DEBUG (~0)
#define QOS_RAW_DEBUG(level, format, arg...)  \
do{ \
	if (g_qos_dbg & level) \
	{ \
		printk(format, ##arg);\
	} \
}while(0)

#define QOS_HAL_DEBUG_MSG(level, format, arg...) QOS_RAW_DEBUG(level, format, ##arg)

struct qos_dgb_info
{
    char *cmd;
    char *des;
    uint32_t flag;
};

static struct qos_dgb_info dbg_enable_mask_str[] = {
    {"err",      "error level",                     QOS_DEBUG_ERR },
    {"trace",      "debug level",                     QOS_DEBUG_TRACE },
    {"high",      "high level",                     QOS_DEBUG_HIGH },
    /*the last one */
    {"all",       "enable all debug",                QOS_ENABLE_ALL_DEBUG}
};

/** QOS HAL Debug Levels */
enum qos_hal_debug_level {
	/** Message */
	QOS_HAL_DBG_MSG,
	/** Warning */
	QOS_HAL_DBG_WRN,
	/** Error */
	QOS_HAL_DBG_ERR,
	/** Off */
	QOS_HAL_DBG_OFF
};

/** Debug message prefix */
#  define DEBUG_PREFIX        "[qos hal]"

#if defined(WIN32)
#  define QOS_HAL_CRLF  "\r\n"
#else
#  define QOS_HAL_CRLF  "\n"
#endif 

/* #define INCLUDE_DEBUG_SUPPORT */

#ifdef INCLUDE_DEBUG_SUPPORT
extern enum qos_hal_debug_level qos_hal_debug_lvl;

int qos_hal_debug_print_err(const char *format, ...);
int qos_hal_debug_print_wrn(const char *format, ...);
int qos_hal_debug_print_msg(const char *format, ...);

#  define DEBUG_ENABLE_ERR
#  define DEBUG_ENABLE_WRN
#  define DEBUG_ENABLE_MSG

#     ifdef DEBUG_ENABLE_ERR
#        define QOS_HAL_DEBUG_ERR   qos_hal_debug_print_err
#     endif			/* DEBUG_ENABLE_ERR */
#     ifdef DEBUG_ENABLE_WRN
#        define QOS_HAL_DEBUG_WRN   qos_hal_debug_print_wrn
#     endif			/* DEBUG_ENABLE_WRN */
#     ifdef DEBUG_ENABLE_MSG
#        define QOS_HAL_DEBUG_MSG   qos_hal_debug_print_msg
#     endif			/* DEBUG_ENABLE_MSG */

#endif				/* INCLUDE_DEBUG_SUPPORT */

#ifndef STATIC
#if 1
#define STATIC static
#else
#define STATIC /**/
#endif
#endif

#ifndef QOS_HAL_DEBUG_ERR
#  if defined(__GNUC__)
#     define QOS_HAL_DEBUG_ERR(fmt, args...)   while(0){}
#  else
#     define QOS_HAL_DEBUG_ERR   {}
#  endif
#endif

#ifndef QOS_HAL_DEBUG_WRN
#  if defined(__GNUC__)
#     define QOS_HAL_DEBUG_WRN(fmt, args...)   while(0){}
#  else
#     define QOS_HAL_DEBUG_WRN   {}
#  endif
#endif

#ifndef QOS_HAL_DEBUG_MSG
#  if defined(__GNUC__)
#     define QOS_HAL_DEBUG_MSG(fmt, args...)   while(0){}
#  else
#     define QOS_HAL_DEBUG_MSG   printk
#  endif
#endif

#endif
