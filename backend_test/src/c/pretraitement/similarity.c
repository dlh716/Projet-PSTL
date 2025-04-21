#include "similarity.h"
#include "../global.h"

int mode_similitude = 0;
int global_histogram[NUM_BINS];

void mean_similitude_job(void *args) {
    struct mean_similitude_args *data = (struct mean_similitude_args*) args;
    double somme = 0.0;
    int count = 0;
    int *local_histogram = data->thread_histograms[data->thread_id];
    
    printf("Thread %d row %d à %d\n", data->thread_id, data->start_row, data->end_row);
    
    for (int i = data->start_row; i < data->end_row; i++) {
        for (int j = i + 1; j < data->num_rows; j++) {
            double similarity = 0.0;
            switch (data->choice) {
                case 0:  // Corrélation
                    similarity = correlation_similarity(i, j);
                    break;
                case 1:  // Distance Cosinus
                    similarity = cosine_similarity(i, j);
                    break;
                case 2:  // Distance Euclidienne
                    similarity = euclidean_distance(i, j);
                    break;
                case 3:  // Norme L1
                    similarity = L1_norm(i, j);
                    break;
                case 4:  // Norme Linf
                    similarity = Linf_norm(i, j);
                    break;
                case 5:  // KL Div
                    similarity = KL_divergence(i, j);
                    break;
                default:
                    printf("Choix non valide.\n");
                    similarity = 0.0;
                    break;
            }

            similarity_matrix[i][j] = similarity;
            similarity_matrix[j][i] = similarity;
            
            int new_index = incr_or_max(data->counter, data->size_similarities);
            if (new_index < data->size_similarities) {
                data->similarities[new_index] = similarity;
            }
            
            if (!isnan(similarity)) {
                somme += similarity;
            }
            count++;
            
            double scaled_similarity = (data->choice == 0 || data->choice == 1) 
                ? (similarity + 1.0) / 2.0 
                : similarity;
            
            int bin_index = (int)(scaled_similarity * NUM_BINS);
            if (bin_index >= NUM_BINS) bin_index = NUM_BINS - 1;
            if (bin_index < 0) bin_index = 0;
            local_histogram[bin_index]++;
        }
    }

    data->res[data->thread_id] = somme;
    data->cpt[data->thread_id] = count;
    printf("Thread %d somme = %f count = %d\n", data->thread_id, somme, count);
}

double calculate_mean_similitude_parallel(int choice, double* similarities) {
    int num_threads = pool.nb_threads;
    double *res = calloc(num_threads, sizeof(double));
    int *cpt = calloc(num_threads, sizeof(int));
    int histogram[NUM_BINS] = {0};

    int **thread_histograms = calloc(num_threads, sizeof(int *));
    for (int t = 0; t < num_threads; t++) {
        thread_histograms[t] = calloc(NUM_BINS, sizeof(int));
    }

    _Atomic int counter = 0;
    int size_similarities = num_rows * (num_rows - 1) / 2; 

    int rows_per_thread = num_rows / num_threads;
    int remaining_rows = num_rows % num_threads;

    for (int t = 0; t < num_threads; t++) {
        struct mean_similitude_args *args = malloc(sizeof(struct mean_similitude_args));
        args->choice = choice;
        args->thread_id = t;
        args->start_row = t * rows_per_thread;
        args->end_row = (t + 1) * rows_per_thread;
        args->num_rows = num_rows;
        args->size_similarities = size_similarities;
        args->res = res;
        args->cpt = cpt;
        args->global_histogram = histogram;
        args->thread_histograms = thread_histograms;
        args->similarities = similarities;
        args->counter = &counter; 

        if (t == num_threads - 1) {
            args->end_row += remaining_rows; 
        }

        struct Job task;
        task.j = mean_similitude_job;
        task.args = args;
        submit(&pool, task);
    }

    while (GetOp(&pool) < num_threads) {}

    for (int t = 0; t < num_threads; t++) {
        for (int bin = 0; bin < NUM_BINS; bin++) {
            histogram[bin] += thread_histograms[t][bin];
        }
        free(thread_histograms[t]);
    }
    free(thread_histograms);

    double somme = 0.0;
    int count = 0;
    for (int t = 0; t < num_threads; t++) {
        somme += res[t];
        count += cpt[t];
    }

    printf("Histogram of Similarities (Normalized):\n");
    for (int bin = 0; bin < NUM_BINS; bin++) {
        double bin_start = (choice == 0 || choice == 1) ? -1.0 + (2.0 * bin) / NUM_BINS : (double)bin / NUM_BINS;
        double bin_end = (choice == 0 || choice == 1) ? -1.0 + (2.0 * (bin + 1)) / NUM_BINS : (double)(bin + 1) / NUM_BINS;
        printf("Bin [%.2f, %.2f): %d\n", bin_start, bin_end, histogram[bin]);
        global_histogram[bin] = histogram[bin]; 
    }

    return (count > 0) ? somme / count : 0.0;
}

// Calcul de la corrélation entre deux vecteurs
double correlation_similarity(int i, int j) {
    double mean_i = 0.0, mean_j = 0.0;
    for (int k = 0; k < num_columns; k++) {
        mean_i += data[i][k];
        mean_j += data[j][k];
    }
    mean_i /= num_columns;
    mean_j /= num_columns;

    double num = 0.0, den_i = 0.0, den_j = 0.0;
    for (int k = 0; k < num_columns; k++) {
        double di = data[i][k] - mean_i;
        double dj = data[j][k] - mean_j;
        num += di * dj;
        den_i += di * di;
        den_j += dj * dj;
    }

    return (num / sqrt(den_i * den_j));
}

// Calcul de la norme L1 (somme des distances absolues)
double L1_norm(int i, int j) {
    double sum = 0.0;
    for (int k = 0; k < num_columns; k++) {
        sum += fabs(data[i][k] - data[j][k]);
    }
    return 1/(1+sum);
}

// Calcul de la norme Linf (distance maximale entre les composantes des vecteurs)
double Linf_norm(int i, int j) {
    double max_diff = 0.0;
    for (int k = 0; k < num_columns; k++) {
        double diff = fabs(data[i][k] - data[j][k]);
        if (diff > max_diff) {
            max_diff = diff;
        }
    }
    return 1/(1+max_diff);
}

// Calcul de la distance cosinus
double cosine_similarity(int i, int j) {
    double num = 0.0, den_i = 0.0, den_j = 0.0;
    for (int k = 0; k < num_columns; k++) {
        num += data[i][k] * data[j][k];
        den_i += data[i][k] * data[i][k];
        den_j += data[j][k] * data[j][k];
    }
    return num / (sqrt(den_i) * sqrt(den_j));
}

// Fonction pour calculer la divergence KL entre deux vecteurs normalisés
double KL_divergence(int i, int j) {
    double kl_div = 0.0;
    double p[num_columns];
    double q[num_columns];

    // Copier les lignes i et j dans des vecteurs locaux pour les normaliser
    for (int k = 0; k < num_columns; k++) {
        p[k] = data[i][k];
        q[k] = data[j][k];
    }

    // Normaliser les deux vecteurs
    normalize_vector(p, num_columns);
    normalize_vector(q, num_columns);

    // Calcul de la divergence KL
    for (int k = 0; k < num_columns; k++) {
        if (p[k] > EPSILON && q[k] > EPSILON) {
            kl_div += p[k] * log(p[k] / q[k]);
        }
    }
    
    return 1/(1+kl_div);
}

// Calcul de la distance euclidienne
double euclidean_distance(int i, int j) {
    double sum = 0.0;
    for (int k = 0; k < num_columns; k++) {
        double diff = data[i][k] - data[j][k];
        sum += diff * diff;
    }
    return 1/(1+sum);
}

// Function to compute the norm of a vector
double compute_norm(double *vector, int length) {
    double norm = 0.0;
    for (int i = 0; i < length; i++) {
        norm += vector[i] * vector[i];
    }
    return sqrt(norm);
}

// Function to normalize a vector
void normalize_vector(double *vector, int length) {
    double norm = compute_norm(vector, length);
    if (norm > 0) {
        for (int i = 0; i < length; i++) {
            vector[i] /= norm;
        }
    }
}

// Fonction de comparaison pour qsort
int compare_double(const void *a, const void *b) {
    double arg1 = *(const double *)a;
    double arg2 = *(const double *)b;

    if (arg1 < arg2) return -1;
    if (arg1 > arg2) return 1;
    return 0;
}

int dichotomie(double threshold, double* similarities) {

    int size_similarities = num_rows * (num_rows-1) / 2;

    int st = 0;
    int end = size_similarities - 1;
    int mid;

    int i = 0;
    while (  st + 1 < end  ) {
        mid = (st + end) / 2;
        if ( similarities[mid] < threshold ) {
            st = mid;
        } else if ( similarities[mid] >  threshold ) {
            end = mid;
        } else {
            // nombre d'element plus grand que threshold
            return size_similarities - mid;
        }
        i += 1;
    }

    return size_similarities - mid;
} 

void calculate_threshold(int choice, int N, double * threshold, double * anti_threshold, double* similarities) {

    if ( choice < 0 || choice > 5 ) {
      perror("Choix non valide");
        exit(EXIT_FAILURE);
    }

    int size_similarities = num_rows * (num_rows - 1) / 2;

    // Tri des similarités pour la dichotomie
    qsort(similarities, size_similarities, sizeof(double), compare_double);
    // Utilisation de la dichotomie pour trouver le seuil
    double low = similarities[0];
    double high = similarities[size_similarities - 1];
    *threshold = 0.5 * (low + high);
    while ((high-low) >= 0.000001) {
        // combre d'arête plus grand que *threshold
        int current_edges = dichotomie(*threshold, similarities);

        // Comparer avec N et ajuster le seuil
        if (current_edges < N-100) {
            high = *threshold;
            *threshold = 0.5 * (low + *threshold);  // Dichotomie vers le bas
        } else if (current_edges > N+100) {
            low = *threshold;
            *threshold = 0.5 * (*threshold + high);
        } else { printf("seuil trouvé \n");
	         fflush(stdout);
            break;  // On a trouvé le seuil exact
        }
    }

     low = similarities[0];
     high = similarities[size_similarities - 1];
     *anti_threshold = 0.5 * (low + high);
    // TODO Potentiel boucle infini ici
    while ((high-low) >= 0.000001) {
        // combre d'arête plus petite qu'anti threshold
        int current_edges = size_similarities - dichotomie(*anti_threshold, similarities) + 1;

        // Comparer avec N et ajuster le seuil
        if (current_edges < N-100) {
            low = *anti_threshold;
            *anti_threshold = 0.5 * (*anti_threshold + high);
//printf("%lf",antiseuil);  // Dichotomie vers le haut
        } else if (current_edges > N+100) {
            high = *anti_threshold;
//printf("%lf",antiseuil);
            *anti_threshold = 0.5 * (low + *anti_threshold);  // Dichotomie vers le bas
        } else {
	        fflush(stdout);
            break;  // On a trouvé le seuil exact
        }
    }

}