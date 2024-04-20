#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int main() {
    int a =100;
    printf("a=%d\n",a);
    const int MATRIX_SIZE = 10;
    int aa =10;
    printf("aa=%d\n",aa);
    int matrix_a[MATRIX_SIZE][MATRIX_SIZE];
    int aaa =10;
    printf("aaa=%d\n",aaa);
    int matrix_b[MATRIX_SIZE][MATRIX_SIZE];
    int aaaa =10;
    printf("aaaa=%d\n",aaaa);
    int result_matrix[MATRIX_SIZE][MATRIX_SIZE];
    int aaaaa =10;
    printf("aaaaa=%d\n",aaaaa);

    // Initialize matrices with random values
    // srand(time(NULL));
    int i = 0, j = 0, k = 0;
    while (i < MATRIX_SIZE) {
        j = 0;
        while (j < MATRIX_SIZE) {
            matrix_a[i][j] = 111;//(double)rand() / RAND_MAX;
            matrix_b[i][j] = 121212;//(double)rand() / RAND_MAX;
            j++;
        }
        i++;
    }

    // Measure computation time
    clock_t start_time = clock();

    // Perform heavy computation
    // perform_computation(matrix_a, matrix_b, result_matrix);
    i = 0, j = 0, k = 0;

    while (i < MATRIX_SIZE) {
        j = 0;
        while (j < MATRIX_SIZE) {
            result_matrix[i][j] = 0.0;
            k = 0;
            while (k < MATRIX_SIZE) {
                result_matrix[i][j] += matrix_a[i][k] * matrix_b[k][j];
                k++;
            }
            j++;
        }
        i++;
    }

    // Calculate computation time
    clock_t end_time = clock();
    double computation_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;

    // Print computation time
    printf("Computation time: %.6f seconds\n", computation_time);

    return 0;
}
