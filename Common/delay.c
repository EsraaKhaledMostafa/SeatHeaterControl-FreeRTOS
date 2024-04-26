/*
 * delay.c
 *
 *  Created on: Apr 14, 2024
 *      Author: DELL
 */

#include "delay.h"

void Delay_MS(unsigned long long n)
{
    volatile unsigned long long count = 0;
    while(count++ < (NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND * n) );
}



