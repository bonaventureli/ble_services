/**
  ******************************************************************************
  * @file    stdin.h
  * @author  lPanda Team
  * @version V1.0.0
  * @date    06-January-2018
  * @brief   type define
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STDIN_H
#define __STDIN_H

#ifdef __cplusplus
	extern "C" {
#endif

  
/* Includes ------------------------------------------------------------------*/

typedef unsigned char         u8_t;
typedef unsigned short int    u16_t;
typedef unsigned int          u32_t;
typedef unsigned long long    u64_t;

typedef signed char           s8_t;
typedef signed short int      s16_t;
typedef signed int            s32_t;
typedef signed long long      s64_t;


typedef u32_t               u_unit_t;   	/**< 无符号 MCU访问数据单元 */
typedef s32_t               s_unit_t;   	/**< 有符号 MCU访问数据单元 */

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus, BitStatus, BitAction;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;

typedef enum {LOW = 0, HIGH = !LOW} LevelState;

#ifndef NULL
#define NULL                (void *)0
#endif

#ifdef ITL_FUNC_OPEN
#define ITL_FUNC
#deifne ITL_DEF

#else
#define ITL_FUNC        static
#define ITL_DEF         static
#endif

#define EN_INT()          __EI()
#define DI_INT()          __DI()

typedef void (*P_GENERAL_CB)(u_unit_t addr, u_unit_t len);

#if 0  //????????
/** 布尔型定义 */
typedef enum {
	FALSE = 0, 	/**< 假 */
	TRUE	    /**< 真 */
} bool_t;

#ifndef bool
typedef bool_t bool;
#endif
#endif

/** 运行状态 */
typedef enum {
  STOP = 0,         /**< 停止 */
  START,            /**< 启动 */
} run_status_t;

/** 运行模式 */
typedef enum {
    SINGLE = 0,     /**< 单一模式 */
    SUCCESSION,     /**< 连续模式 */
} run_mode_t;

/** 时间状态 */
typedef enum {
    NO_OUTTIME = 0, /**< 没有超时 */
    OUTTIME,        /**< 超时 */
} time_status_t;


/**
 * 断言失败处理
 *   
 * @param [file] 文件名
 * @param [line] 行号
 *
 * @return 无
 */
extern void assert_failed(const s8_t *file, u32_t line);


/** 断言失败处理接口 */
#define assert_param(expr) ((expr) ? (void)0 : \
    assert_failed((const uint8_t *)__FILE__, __LINE__))


/* 返回值类型定义 */
typedef enum {
    RV_NOSUPPORT = -7,  /**< 不支持 */
	RV_TIMEOUT = -6,	/**< 超时 */
	RV_SLEEPING = -5,	/**< 休眠请求已响应 */
	RV_CONFIG_ERR = -4,	/**< 配置参数错误 */
	RV_SENDING = -3,	/**< 发送请求已响应 */
	RV_PARAM_ERR = -2,	/**< 参数错误 */
	RV_FAILED = -1,		/**< 失败 */
	RV_SUCCESS = 0,		/**< 成功 */
} rv_t;


/** 获取p_data的第n个数据 */
#define GET_BYTE(p_data, n) \
	(((p_data) >> (8 * (n))) & 0xFF)
	
/** p1与p2合成两字节 */
#define ADD16(p1, p2) \
	(((uint16_t)(p1) << 8) | ((uint16_t)p2))
	
/** p1/p2/p3/p4合成四字节 */
#define ADD32(p1, p2, p3, p4) \
	(((uint32_t)(p1) << 24) | ((uint32_t)(p2) << 16) | \
	((uint32_t)(p3) << 8) | ((uint32_t)p4))

/** 取x/y的最小值 */
#define MIN(x, y) \
	((x) < (y) ? (x) : (y))

/** 取x/y的最大值 */
#define MAX(x, y) \
	((x) > (y) ? (x) : (y))

/** 取x/y的差值 */
#define ABS_SUB(x, y) \
	(((x) >= (y)) ? (x - y) : (y - x))

/** data的第n位置1 */
#define SET_BIT(data, n) \
	((uint32_t)(data) | (uint32_t)(1 << (n)))

/** data的第n位清0 */
#define CLEAR_BIT(data, n) \
	((uint32_t)(data) & (uint32_t)(~(1 << (n))))

/** Get Bit Value */
#define GET_BIT(data, n) \
	((uint32_t)(data) & (uint32_t)(1 << (n)))


#ifdef __cplusplus
}
#endif

#endif /* __STDIN_H */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) *****END OF FILE****/
