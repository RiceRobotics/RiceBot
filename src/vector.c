/*
 * vector.c
 *
 *  Created on: Jan 24, 2015
 *      Author: Cannon
 */

#include "vector.h"

Motor_vector* init_Motor_vector() {
	Motor_vector* vect = malloc(sizeof(Motor_vector));
	vect->elem_total = 10;
	vect->elem_current = 0;
	return vect;
}

int Motor_vector_append(Motor_vector* vect, Motor* element) {
	vect->data[vect->elem_current] = element;
	++vect->elem_current;
	if(vect->elem_current >= vect->elem_total) {
		Motor* new_data = realloc(vect->data, (vect->elem_total * 2) * sizeof(Motor));
		if(new_data) {
			vect->data = new_data;
			vect->elem_total *= 2;
		} else {
			printf("Error allocating memory");
			free(vect->data);
			return 0;
		}
	}
	return 1;
}

/*
Motor* Motor_vector_get(Motor_vector* vect, int index) {

}
*/
