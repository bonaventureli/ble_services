/******************************************************************************

******************************************************************************/
#include "convert.h"

/*
 * 数据转义规则：采用0x7D作为转义字符，对0x7E和0x7F以及0x7D进行转义
 * 0x7E转义为0x7D、0x5E，
 * 0x7F转义为0x7D、0x5F，
 * 0x7D转义为0x7D、0x5D
 */


/*****************************************************************************
 函 数 名: Escape_frame_data
 功能描述: 数据转义
 输入参数: uint8_t* input    : 输入数据buffer指针
           uint32_t ilen     : 输入数据长度
           uint8_t* output   : 输出数据buffer指针
           uint32_t* olen    : 输出数据长度
 输出参数: uint8_t
 返 回 值: 1：转义失败 0：转义成功
 调用函数: 
 被调函数: 
 

*****************************************************************************/
uint8_t Escape_frame_data(uint8_t* input, uint32_t ilen, uint8_t* output, uint32_t* olen)
{
    int i=0,index=0;
    int count=0;
	 uint8_t replacedata[3][2]={{0x7D,0x5E},{0x7D,0x5F},{0x7D,0x5D}};

    if(input == NULL || ilen <= 0)
    {
        return 1;
    }
    
    for(i=0; i<ilen; i++)
    {
        if(input[i] == 0x7E)
        {
            memcpy(output+index,replacedata[0],2);
            index+=2;
            count++;
        }
        else if(input[i] == 0x7F)
        {
            memcpy(output+index,replacedata[1],2);
            index+=2;
            count++;
        }
        else if(input[i] == 0x7D)
        {
            memcpy(output+index,replacedata[2],2);
            index+=2;
            count++;
        }
        else
        {
            output[index]=input[i];
            index++;
        }
    }
    *olen = index;

    return 0;
}


/*****************************************************************************
 函 数 名: Reduction_frame_data
 功能描述: 数据反转义
 输入参数: uint8_t* input    : 输入数据buffer指针
           uint32_t ilen     : 输入数据长度
           uint8_t* output   : 输出数据buffer指针
           uint32_t* olen    : 输出数据长度
 输出参数: uint8_t
 返 回 值: 1：反转义失败 0：反转义成功
 调用函数: 
 被调函数: 
 

*****************************************************************************/
uint8_t Reduction_frame_data(uint8_t* input, uint32_t ilen, uint8_t* output, uint32_t* olen)
{
    int i=0, index=0;
    int count = 0;
    uint8_t* pdata=NULL;
 
    if(input == NULL || (ilen-2) <= 0)
    {
        return 1;
    }
 
    pdata = input;
    for(i=0; i < ilen; i++)
    {
        if(pdata[i] == 0x7D)
        {
       	   count++;
 
            if(pdata[i+1] == 0x5E)
            {
           	 output[index]=0x7E;
            }
            else if(pdata[i+1] == 0x5F)
            {
           	 output[index]=0x7F;
            }
            else if(pdata[i+1] == 0x5D)
            {
           	 output[index]=0x7D;
            }
            i++;
            index++;
        }
        else
        {
       	 output[index]=pdata[i];
         index++;
        }
    }
 
     *olen = index;
     return 0;
}
