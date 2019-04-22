#ifndef __MLL_H
#define __MLL_H
#define pixel_num 6000
#define layer0_node 16
#define layer1_node 8
#define output_node 2

#include <stm32f4xx.h>
u8 pixel[pixel_num];
float layer0[layer0_node];
float layer1[layer1_node];
float output[output_node];

s8 weights0[pixel_num][layer0_node];
float weights1[layer0_node][layer1_node];
float weights2[layer1_node][output_node];

float layer0_shift[layer0_node];
float layer1_shift[layer1_node];
float output_shift[output_node];	 

u8 random_for_s8(void);
u8 random_for_float(void);

float Logistic(float x);
void init_weights(void);
u8 mlforward(void);
void mlback(int x);

#endif