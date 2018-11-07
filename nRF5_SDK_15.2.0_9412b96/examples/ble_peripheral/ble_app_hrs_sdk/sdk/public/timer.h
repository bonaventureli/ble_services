/**
  ******************************************************************************
  * @file    timer.h
  * @author  lPanda Team
  * @version V1.0.0
  * @date    14-March-2018
  * @brief   timer
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/


#ifndef __TIMER_H
#define __TIMER_H


#ifdef __cplusplus
	extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdin.h"

/** 函数类型 */
typedef enum { 
    GENERAL = 0,    /**< 普通函数类型 */
    FSM,            /**< 状态机类型 */        
} function_t;


/** Timer ID type */
typedef enum {
    TID_USER = 0,

    /** 用户自行添加任务ID 一个ID做一个任务 */
    TID_CAN,
    TID_BT_COMM,
    TID_MAX
} timer_id_t;

/** Timer information tpye */
struct timer_info_t {
    time_status_t   time_status;    /**< 时间状态 */
    run_status_t    run_status;     /**< 运行状态 */
    run_mode_t      run_mode;       /**< 运行模式 */
    
    void            *fp;            /**< 函数指针 */
    void            *p_data;        /**< 数据指针 */

    u32_t           data_size;      /**< 数据大小 */
    u32_t           timer_counter;  /**< 定时器计数 */
    u32_t           reload_counter; /**< 重载计数器 */
    function_t      type;           /**< 函数类型 */
};


/**
 * 启动FSM任务
 *   
 * @param [id] 定时器ID
 * @param [fp] 函数指针
 *
 * @return 返回RV_SUCCESS成功，否则失败。
 */
rv_t TimerStartFSM_Task(u32_t id, void *fp, run_mode_t mode, u32_t ms);

/**
 * 启动普通任务
 *   
 * @param [id] 定时器ID
 * @param [fp] 函数指针
 * @param [p_data] 数据指针
 * @param [size] 数据大小
 * @param [mode] 运行模式
 * @param [ms] 时间
 *
 * @return 返回RV_SUCCESS成功，否则失败。
 */
rv_t TimerStartGeneralTask(u32_t id, P_GENERAL_CB fp, void *p_data, 
    u32_t size, run_mode_t mode, u32_t ms);

/**
 * 复位定时器
 *   
 * @param [id] 定时器ID
 *
 * @return 返回RV_SUCCESS成功，否则失败。
 */
rv_t TimerResetTask(u32_t id);

/**
 * 复位定时器
 *   
 * @param [id] 定时器ID
 *
 * @return 返回RV_SUCCESS成功，否则失败。
 */
rv_t TimerStopTask(u32_t id);

/**
 * 杀掉定时器
 *   
 * @param [id] 定时器ID
 *
 * @return 返回RV_SUCCESS成功，否则失败。
 */
rv_t TimerKillTask(u32_t id);

/**
 * 定时器轮询进程
 *   
 *
 * @return 返回RV_SUCCESS成功，否则失败。
 */
void TimerPollProcesses(void);

/**
 * 初始化定时器
 *   
 *
 * @return 返回RV_SUCCESS成功，否则失败。
 */
void TimerInit(void);


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


