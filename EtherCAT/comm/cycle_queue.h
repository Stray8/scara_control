
#ifndef _MASTER_COMM_CYCLE_QUEUE_H_
#define _MASTER_COMM_CYCLE_QUEUE_H_

//typedef struct cycle_queue * cycle_queue_t;


typedef struct 
{
    unsigned char * buffer;

    unsigned int front;

    unsigned int element_count;
    unsigned int element_size;
    unsigned int element_count_max;

    int overwrite_when_full;
}cycle_queue;


/// @brief 创建一个循环队列
/// @param [in] element_size        元素大小
/// @param [in] element_count_max   元素最大个数
/// @param [in] overwrite_when_full 满之后自动覆盖
/// @return 返回循环队列实例
void cycle_queue_create(cycle_queue * queue, unsigned char * buf, unsigned int element_size, unsigned int element_count_max, int overwrite_when_full);

/// 销毁一个循环队列
void cycle_queue_destroy(cycle_queue * queue);

/// @brief 在队列尾部加入一个新元素
/// @param [in] queue   队列对象
/// @param [in] element 元素指针
/// @return 添加成功返回0，否则-1
/// @note 满之后的行为取决于cycle_queue_create的overwrite_when_full参数
int cycle_queue_push(cycle_queue * queue, const void * element);

/// 添加一个空元素，返回其缓冲区
unsigned char * cycle_queue_push_empty(cycle_queue * queue);

/// @brief 获取队列头部数据
/// @param [in]  queue    队列对象
/// @param [out] element  存放获取到的元素数据的缓冲区
/// @param [in]  pop      指定是否从队列中删除
/// @return 成功返回0，否则返回-1
int cycle_queue_peek(cycle_queue * queue, void * element, int pop);

/// @brief 从队列头部取出一个元素
/// @param [in]  queue    队列对象
/// @param [out] element  存放获取到的元素数据的缓冲区
/// @return 成功返回0，否则返回-1
int cycle_queue_pop(cycle_queue * queue, void * element);

/// 判断队列是否为空，空返回1，非空返回0
int cycle_queue_is_empty(cycle_queue * queue);

/// 判断队列是否为满，满返回1，非满返回0
int cycle_queue_is_full(cycle_queue * queue);

/// 获取队列当前元素个数
unsigned int cycle_queue_element_count(cycle_queue * queue);

/// 获取队列最大元素个数
unsigned int cycle_queue_element_count_max(cycle_queue * queue);

#endif
