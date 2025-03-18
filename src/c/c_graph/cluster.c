#include "cluster.h"
#include "../global.h"

int communities[MAX_NODES]; // Stocke les communautés détectées par Louvain

int clusters[MAX_NODES];
float cluster_colors[MAX_NODES][3];
double centers[MAX_NODES][2];
Cluster *cluster_nodes = NULL;  // Tableau de clusters
int n_clusters;

double epsilon = 0.1;
int espacement = 1;

// modifiable par utilisateur
double repulsion_coeff = 1;
int saut = 10;
int mode = 0;

// Assigner des noeuds aux clusters et mettre à jour les centres en utilisant l'algorithme k-means
void kmeans_iteration(int num_points, int num_clusters, int *labels, double centers[][2], double Lx, double Ly) {

    // Modification pour utiliser moins de memoire et eviter un seg fault
    int* counts = (int*) malloc(sizeof(int) * num_nodes);
    double ** new_centers = (double**) malloc(sizeof(double*) * num_clusters);  // Stocker les nouveaux centres calculés

    for (int i = 0; i < num_nodes; ++i){
        counts[i] = 0;
    } 

    for (int i = 0; i < num_clusters; ++i)
    {
        new_centers[i] = (double*) malloc(sizeof(double) * 2);
        new_centers[i][0] = 0.;
        new_centers[i][1] = 1.;
    }

    // Assigner chaque point au cluster le plus proche et mettre à jour les centres
    for (int i = 0; i < num_points; i++) {
        double min_dist = DBL_MAX;
        int best_cluster = 0;

        for (int j = 0; j < num_clusters; j++) {
            Point dir;
            toroidal_vector(&dir, vertices[i], (Point){centers[j][0], centers[j][1]});
            double dist = (dir.x * dir.x + dir.y * dir.y);

            if (dist < min_dist) {
                min_dist = dist;
                best_cluster = j;
            }
        }

        labels[i] = best_cluster;

        // Ajuster les coordonnées du point pour qu'elles soient proches du centre du cluster
        double adjusted_x = vertices[i].x;
        double adjusted_y = vertices[i].y;

        while (adjusted_x - centers[best_cluster][0] > Lx / 2) adjusted_x -= Lx;
        while (centers[best_cluster][0] - adjusted_x > Lx / 2) adjusted_x += Lx;
        while (adjusted_y - centers[best_cluster][1] > Ly / 2) adjusted_y -= Ly;
        while (centers[best_cluster][1] - adjusted_y > Ly / 2) adjusted_y += Ly;

        new_centers[best_cluster][0] += adjusted_x;
        new_centers[best_cluster][1] += adjusted_y;
        counts[best_cluster]++;
    }

    // Mise à jour des centres de clusters en fonction des nouvelles assignations
    for (int i = 0; i < num_clusters; i++) {
        if (counts[i] > 0) {
            centers[i][0] = new_centers[i][0] / counts[i];
            centers[i][1] = new_centers[i][1] / counts[i];

            // Ramener les centres dans l'espace torique
            while (centers[i][0] < -Lx / 2) centers[i][0] += Lx;
            while (centers[i][0] > Lx / 2) centers[i][0] -= Lx;
            while (centers[i][1] < -Ly / 2) centers[i][1] += Ly;
            while (centers[i][1] > Ly / 2) centers[i][1] -= Ly;
        }
    }

    for (int i = 0; i < num_clusters; ++i)
    {
        free(new_centers[i]);
    }
    free(new_centers);
    free(counts);
}

// probablement privé utilisée dans update_positions
// Étape 2 : Forces de répulsion intra-cluster
void repulsion_intra_clusters(Point* forces, double FMaxX, double FMaxY)
{

    for (int cluster = 0; cluster < n_clusters; cluster++) {
        int size = cluster_nodes[cluster].size;
        for (int i = 0; i < size; i++) {
            int node_i = cluster_nodes[cluster].nodes[i];
    
            Point pi = vertices[i];
            for (int j = i + 1; j < size; j++) {
                int node_j = cluster_nodes[cluster].nodes[j];
                Point dir;
                toroidal_vector(&dir, pi, vertices[i]);
    
                double dist_squared = dir.x * dir.x + dir.y * dir.y;
                if (dist_squared > seuilrep) { // Assume a minimum distance to avoid division by zero
                    //double dist = sqrt(dist_squared);
                    
                    double rep_force;
                    if ( mode == 1 ) { 
                        rep_force = repulsion_coeff/ dist_squared*dist_squared;
                    } else if (mode == 2 && communities[i] != communities[j]) {//printf("extra repulsion %d, %d \n",i,j);
                        rep_force = 100000*repulsion_coeff*(node_degrees[i]+1)*(node_degrees[j]+1) / dist_squared;
                    } else { // mode == 0 est le mode par defaut
                            rep_force = repulsion_coeff*(node_degrees[i]+1)*(node_degrees[j]+1) / dist_squared;
                    } 
                                        
                    forces[node_i].x -= dir.x * rep_force;
                    forces[node_i].y -= dir.y * rep_force;
                    forces[node_j].x += dir.x * rep_force;
                    forces[node_j].y += dir.y * rep_force;
                } else {
                    double rep_force = repulsion_coeff / seuilrep;
                                        
                    forces[node_i].x -= dir.x * rep_force;
                    forces[node_i].y -= dir.y * rep_force;
                    forces[node_j].x += dir.x * rep_force;
                    forces[node_j].y += dir.y * rep_force;
                }
            }
                // capper les forces d'attractions
        forces[node_i].x = fmax(fmin(forces[node_i].x, FMaxX), -FMaxX);
        forces[node_i].y = fmax(fmin(forces[node_i].y, FMaxY), -FMaxY);
    
        }
    }


}

// etape 4 dans update_positions
void update_clusters()
{
    if (iteration % (saut * (1+espacement)) == 0) {
        int centers_converged = 0;

        while (centers_converged == 0) {
            double old_centers[MAX_NODES][2];
            memcpy(old_centers, centers, sizeof(centers));

            kmeans_iteration(num_nodes, n_clusters, clusters, centers, Lx, Ly);

            centers_converged = 1;
            for (int i = 0; i < n_clusters; i++) {
                double dx = centers[i][0] - old_centers[i][0];
                double dy = centers[i][1] - old_centers[i][1];
                if ((dx * dx + dy * dy) > epsilon) {
                    centers_converged = 0;
                    break;
                }
            }
        }
        clear_clusters();

        for (int i = 0; i < num_nodes; i++) {
            int best_cluster = clusters[i];
            add_node_to_cluster(best_cluster, i);
        }
    }
}

void clear_clusters() {
    for (int i = 0; i < n_clusters; i++) {
        cluster_nodes[i].size = 0;
    }
}

void reinitialize_clusters(int new_num_clusters) {
    if (new_num_clusters != n_clusters) {
        free_clusters();
        init_clusters(new_num_clusters);
    } else {
        clear_clusters();
    }
    initialize_centers();
    assign_cluster_colors();
    n_clusters = new_num_clusters;
}

void free_clusters() {
    if (cluster_nodes != NULL) {
        for (int i = 0; i < n_clusters; i++) {
            if (cluster_nodes[i].nodes != NULL) {
                free(cluster_nodes[i].nodes);
                cluster_nodes[i].nodes = NULL;
            }
        }
        free(cluster_nodes);
        cluster_nodes = NULL;
    }
}

// Générer un point aléatoire dans le plan
void random_point_in_plane(Point *p) {
    p->x = (rand() / (double)RAND_MAX) * Lx - Lx / 2;
    p->y = (rand() / (double)RAND_MAX) * Ly - Ly / 2;
}

void initialize_centers(){
    for (int i = 0; i < n_clusters; i++) {
        random_point_in_plane((Point *)centers[i]);
    }
}

// Assigner des couleurs aux clusters
void assign_cluster_colors() {
    for (int i = 0; i < n_clusters; i++) {
        cluster_colors[i][0] = (float)rand() / RAND_MAX;
        cluster_colors[i][1] = (float)rand() / RAND_MAX;
        cluster_colors[i][2] = (float)rand() / RAND_MAX;
    }
}

void init_clusters(int num_clusters) {
    int estimated_capacity = (num_nodes / num_clusters) + 1;
    cluster_nodes = (Cluster *)malloc(num_clusters * sizeof(Cluster));
    for (int i = 0; i < num_clusters; i++) {
        cluster_nodes[i].nodes = (int *)malloc(estimated_capacity * sizeof(int));
        cluster_nodes[i].size = 0;
        cluster_nodes[i].capacity = estimated_capacity;
    }
}

void add_node_to_cluster(int cluster_id, int node) {
    Cluster *cluster = &cluster_nodes[cluster_id];
    if (cluster->size == cluster->capacity) {
        cluster->capacity *= 2;
        cluster->nodes = (int *)realloc(cluster->nodes, cluster->capacity * sizeof(int));
    }
    cluster->nodes[cluster->size++] = node;
}