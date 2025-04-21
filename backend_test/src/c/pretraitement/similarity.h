#ifndef FORCEATLAS_SIMILITUDE_H
#define FORCEATLAS_SIMILITUDE_H

#include <stdatomic.h>
#include "../concurrent/Pool.h"

// Prototypes des fonctions de calcul des mesures de similarité
double correlation_similarity(int i, int j);
double cosine_similarity(int i, int j);
double euclidean_distance(int i, int j);
double L1_norm(int i, int j);
double Linf_norm(int i, int j);
double KL_divergence(int i, int j);

double compute_norm(double *vector, int length);
void normalize_vector(double *vector, int length);
void calculate_threshold(int choice, int N, double * threshold, double * anti_threshold, double* similarities);
double calculate_mean_similitude_parallel(int choice, double* similarities);

struct mean_similitude_args {
    int choice;
    int thread_id;
    int start_row;   
    int end_row;     
    int num_rows;   
    int size_similarities;
    double *res;
    int *cpt;
    int *global_histogram;
    int **thread_histograms;
    double *similarities;
    _Atomic int *counter; 
};
#endif