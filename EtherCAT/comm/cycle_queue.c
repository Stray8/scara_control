//############################################################
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//EtherCAT从站学习板
//Author：廷华电子设计
//淘宝店铺: https://shop461235811.taobao.com/
//我的博客：https://blog.csdn.net/zhandouhu/article/category/9455918
//############################################################
#include <stdio.h> 
#include <stdlib.h>
#include <string.h>

#include "cycle_queue.h"


void cycle_queue_create(cycle_queue * queue, unsigned char * buf, unsigned int element_size, unsigned int element_count_max, int overwrite_when_full) {
    //cycle_queue_t queue = calloc(1, sizeof(struct cycle_queue));

    if (!queue) {
        return;// NULL;
    }
    queue->buffer = buf;
    //if (!(queue->buffer = calloc(element_count_max, element_size))) {
    //    free(queue);
    //    return NULL;
    //}

    queue->element_count_max = element_count_max;
    queue->element_size = element_size;
    queue->overwrite_when_full = overwrite_when_full;

    
}

void cycle_queue_destroy(cycle_queue * queue) {
   // if (queue) {
        //free(queue->buffer);
       // free(queue);
   // }
}

int cycle_queue_push(cycle_queue * queue, const void * element) {
    unsigned char * buffer = cycle_queue_push_empty(queue);

    if (buffer && element) {
        memcpy(buffer, element, queue->element_size);
    }

    return buffer ? 0 : -1;
}

unsigned char * cycle_queue_push_empty(cycle_queue * queue) {
    unsigned char * buffer = NULL;

    if (queue) {
        if (queue->element_count == queue->element_count_max) {
            if (!queue->overwrite_when_full) {
                return NULL; 
            }

            buffer = queue->buffer + queue->front * queue->element_size;
            queue->front = (queue->front + 1) % queue->element_count_max;
        }
        else {
            buffer = queue->buffer + ((queue->front + queue->element_count) % queue->element_count_max) * queue->element_size;
            ++queue->element_count;
        }
    }

    return buffer;
}

int cycle_queue_peek(cycle_queue * queue, void * element, int pop) {
    if (queue && queue->element_count > 0) {
        if (element) {
            memcpy(element, queue->buffer + queue->front * queue->element_size, queue->element_size);
        }

        if (pop) {
            queue->front = (queue->front + 1) % queue->element_count_max;
            --queue->element_count;
        }

        return 0;
    }

    return -1;
}

int cycle_queue_pop(cycle_queue * queue, void * element) {
    return cycle_queue_peek(queue, element, 1);
}

int cycle_queue_is_empty(cycle_queue * queue) {
    return !queue || queue->element_count > 0;
}

int cycle_queue_is_full(cycle_queue * queue) {
    return queue ? queue->element_count == queue->element_count_max : 1;
}

unsigned int cycle_queue_element_count(cycle_queue * queue) {
    return queue ? queue->element_count : 0;
}

unsigned int cycle_queue_element_count_max(cycle_queue * queue) {
    return queue ? queue->element_count_max : 0;
}
