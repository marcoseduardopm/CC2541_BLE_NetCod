#ifndef PSEUDO_INVERSE_H
#define PSEUDO_INVERSE_H

//Define here the number of rows of cols for the current matrix




//
#define MAX_ITERATION_COUNT 30   // Maximum number of iterations


void transpose_matrix(double *A, double *result, int row, int col);
void multiply_matrices(double *A, double *B, double *result, int rowA, int colA, int rowB, int colB);

int Singular_Value_Decomposition(double* A, int nrows, int ncols, double* U, 
                      double* singular_values, double* V, double* dummy_array);

void Singular_Value_Decomposition_Inverse(double* U, double* D, double* V,  
                        double tolerance, int nrows, int ncols, double *Astar);

#endif // PSEUDO_INVERSE_H