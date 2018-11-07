/**
  ******************************************************************************
  * @file    queue.h
  * @author  bPanda Team
  * @version V1.0.0
  * @date    22-January-2018
  * @brief   
  ******************************************************************************
  * @attention
  *         
  ******************************************************************************
  */

#ifndef __QUEUE_H
#define __QUEUE_H

/* Includes ------------------------------------------------------------------*/
#include "stdin.h"

#ifdef __cplusplus
	extern "C" {
#endif

/** @defgroup public fsm 
  * @{
  */

#ifdef __cplusplus
}
#endif

struct queue_head_t {
    u32_t            size;       /**< 基数大小 */
    u32_t            count;      /**< 缓冲个数 */
    u32_t            len;        /**< 有效数据长度 [byte = len * size] */
    u32_t            front;      /**< 首位置 */
    u32_t            rear;       /**< 尾位置 */
};

struct queue_t {
    struct queue_head_t head;    /**< 头信息 */
    u8_t             data[1];    /**< 数据缓冲区 */
};


/** Interrupted callback */
typedef void (*P_INT_CB)(void);	

/** New Queue size */
#define NEW_QUEUE_SIZE(size, cnt) \
    (sizeof(struct queue_head_t) + ((size) * (cnt)))

/**
 * 队列注册中断回调函数
 *   
 * @param [enable] 中断使能回调
 * @param [disable] 中断禁止回调
 *
 * @return 空
 */
void QueueRegister(P_INT_CB enable, P_INT_CB disable);


/**
 * 队列输入
 *   
 * @param [queue] 队列指针
 * @param [data] 数据指针
 * @param [size] 输入大小
 * @param [count] 输入个数 
 *
 * @return 大于0返回实际存入的缓冲个数，否则失败。
 */
s32_t QueuePutData(struct queue_t *queue, void *data, u32_t size, 
    u32_t count);


/**
 * 队列输出
 *   
 * @param [queue] 队列指针
 * @param [data] 数据指针
 * @param [size] 输出大小
 * @param [count] 输出个数 
 *
 * @return 大于0返回实际取出的缓冲个数，否则失败。
 */
s32_t QueueGetData(struct queue_t *queue, void *data, u32_t size, 
    u32_t count);

/**
 * 获取有效数据大小
 *   
 * @param [queue] 队列指针
 *
 * @return 大于或等于0返回有效数据大小，否则失败。
 */
s32_t QueueGetDataSize(struct queue_t *queue);


/**
 * 清除队列
 *   
 * @param [queue] 队列指针
 *
 * @return 返回RV_SUCCESS成功，否则失败。
 */
rv_t QueueClearData(struct queue_t *queue);


/**
 * 队列初始化
 *   
 * @param [queue] 队列指针
 * @param [size] 基数大小
 * @param [count] 缓冲区个数 
 *
 * @return 返回RV_SUCCESS成功，否则失败。
 */
rv_t QueueInit(struct queue_t *queue, u32_t size, u32_t count);


#endif /* __FSM_H */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
