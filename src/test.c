/*
 * test.c
 *
 *  Created on: Jan 24, 2015
 *      Author: Cannon
 */

#include "vector.h"
#include <stdio.h>

main() {
	Motor_vector* vect = init_Motor_vector();
	printf("%d\n", sizeof(vect->data));
}
