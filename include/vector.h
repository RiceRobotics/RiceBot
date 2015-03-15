/*
 * vector.h
 *
 *  Created on: Jan 24, 2015
 *      Author: Cannon
 */

#ifndef VECTOR_H_
#define VECTOR_H_

/*
 * Struct representing a dynamic array of int elements.
 */

typedef struct Motor_vector {
	unsigned int elem_total; /* Number of elements currently allocated for, initialized to 10 */
	unsigned int elem_current; /* Current number of elements stored */
	Motor* data[]; /* Pointer to actual array of data */
} Motor_vector;

Motor_vector* init_Motor_vector(); //Initalizes a vector

int Motor_vector_append(element); //Adds an element to the vector. Returns 1 if successful and 0 otherwise

Motor* Motor_vector_get(index); //Returns the element at a given index. Returns -1 if no element at index.


#endif /* VECTOR_H_ */
