#include "cluster.h"
#include "../global.h"

int communities[MAX_NODES]; // Stocke les communautés détectées par Louvain

int clusters[MAX_NODES];

Cluster* grid_list = NULL;
void freeSpatialCell(Cluster* cell, int size) {
    for (int i = 0; i < size; ++i) {
        free(cell[i].nodes);
    }
    free(cell);
}

void add_elem_to_cell(
    Cluster* cell,
    int elem)
{
    if ( cell->size >= cell->capacity ) {
        cell->nodes = (int*) realloc(cell->nodes, cell->capacity * sizeof(int) * 2);
        cell->capacity *= 2;
    } 
    (cell->nodes)[cell->size] = elem;
    ++cell->size;
}

float cluster_colors[MAX_NODES][3];
double centers[MAX_NODES][2];
Cluster *cluster_nodes = NULL;  // Tableau de clusters
int n_clusters;
int kmeans_mode = 0;

double epsilon = 0.1;
int espacement = 1;

int no_overlap = 0;
double initial_node_size = 5.;
double degree_scale_factor = 0.5;

// modifiable par utilisateur
double repulsion_coeff = 10;
int saut = 10;
int mode = 0;

double compute_node_size(int node_index) {
    if ( degree_scale_factor < 0 )
        degree_scale_factor = 0;
    
    return initial_node_size + degree_scale_factor * node_degrees[node_index];
}

double squared_distance(double* center, Point p) {
    double dx = p.x - center[0];
    double dy = p.y - center[1]; 
    return dx * dx + dy * dy;
}

/** This function serves to initialize the centers in the k-means algorithm (It implements the kmeans++ heuristic)
 * @attention Points should be initialized beforehand
 * @param n_clusters : uses the global variable n_clusters
 * @details sequential time complexity: O(NK)
 */
void initialize_centers_plus() {
    // Array allowing us to keep track of distance between points and their nearest centers
    double centers_distance[MAX_NODES] = {DBL_MAX};

    // choose a random vertex of the graph
    int index = rand() % num_nodes;
    // we mark the point index as already chosen
    centers_distance[index] = -1;
    centers[0][0] = vertices[index].x;
    centers[0][1] = vertices[index].y;

    for (int i = 1; i < n_clusters; ++i) {
        double distance_sum = 0.;

        // computing the distance between each points and the set of centers
        for (int j = 0; j < num_nodes; ++j) {
            if ( centers_distance[j] >= 0) {
                double d = squared_distance(centers[i-1], vertices[j]);
                centers_distance[j] = d < centers_distance[j] ? d : centers_distance[j];  
                distance_sum += centers_distance[j];
            }
        }

        // sampling a random point x with propability centers_distance[x] / "dist between points and closest centers" 
        double prob = rand() / (double) RAND_MAX;
        double f_rep = 0.;
        for (int j = 0; j < num_nodes; ++j) {
            if ( centers_distance[j] >= 0 ){
                f_rep += centers_distance[j] / distance_sum;
            }

            if ( prob < f_rep ) {
                centers_distance[j] = -1;
                centers[i][0] = vertices[j].x;
                centers[i][1] = vertices[j].y;
                break;
            }
        }

    }

}

void maintainSpatialGrid() {

    int grid_list_size = 400;
    int grid_length = 20;

    double center_width = Lx * 0.5;
    double center_height = Ly * 0.5;

    double grid_width = Lx / grid_length;
    double grid_height = Ly / grid_length;

    if ( grid_list != NULL ) {
        freeSpatialCell(grid_list, grid_list_size);
    }

    grid_list = (Cluster*) malloc(sizeof(Cluster) * grid_list_size);
    for (int i = 0; i < grid_list_size; ++i) {
        grid_list[i].capacity = 4;
        grid_list[i].size = 0;
        grid_list[i].nodes = (int*) malloc(sizeof(int) * grid_list_size);
    }

    for (int i = 0; i < num_nodes; ++i) {

        if ( vertices[i].deleted == 0 ) {
            double node_size = compute_node_size(i);
            double minX = vertices[i].x - node_size + center_width / 2 < 0 ? 0 : vertices[i].x - node_size + center_width / 2;
            double maxX = vertices[i].x + node_size + center_width / 2 < 0 ? 0 : vertices[i].x + node_size + center_width / 2;
            double minY = vertices[i].y - node_size + center_height / 2 < 0 ? 0 : vertices[i].y - node_size + center_height / 2;
            double maxY = vertices[i].y + node_size + center_height / 2 < 0 ? 0 : vertices[i].y + node_size + center_height / 2;
            int grid_minX = fmax(minX / grid_width, 0);
            int grid_maxX = fmin(maxX / grid_width, grid_length - 1);
            int grid_minY = fmax(minY / grid_height, 0);
            int grid_maxY = fmin(maxY / grid_height, grid_length - 1);


            for (int k = grid_minY; k <= grid_maxY; ++k) {
                for (int j = grid_minX; j <= grid_maxX; ++j) {
                    add_elem_to_cell(&grid_list[k * grid_length + j], i);
                }
            }
        }
    }

}

void grid_clustering(
    int num_points, 
    int num_clusters, 
    int *labels, 
    double Lx, 
    double Ly) {

    int grid_length = (int) sqrt(num_clusters); 

    double center_width = Lx * 0.5;
    double center_height = Ly * 0.5;

    double grid_width = Lx / grid_length;
    double grid_height = Ly / grid_length;

    for (int i = 0; i < num_points; ++i) {
        if ( vertices[i].deleted == 0 ) {
            double x = vertices[i].x + center_width / 2 < 0 ? 0 : vertices[i].x + center_width / 2;
            double y = vertices[i].y + center_height / 2 < 0 ? 0 : vertices[i].y + center_height / 2;
            int grid_i = fmin(x / grid_width, grid_length - 1);
            int grid_j = fmin(y / grid_height, grid_length - 1);

            labels[i] = grid_j * grid_length + grid_i;
        }
    }

}

void noverlap_sequential(double (*forces)[2], int node_index, int grid_index, int* map, double FMaxX, double FMaxY) {

    for (int i = 0; i < grid_list[grid_index].size; ++i) {

        int grid_node = grid_list[grid_index].nodes[i];

        if ( grid_node > node_index && map[grid_node] != node_index ) {

            map[grid_node] = node_index;

            double node_size = compute_node_size(node_index);
            double grid_node_size = compute_node_size(grid_node);

            Point dir;
            toroidal_vector(&dir, vertices[node_index], vertices[grid_node]);
            double dist_squared = dir.x * dir.x + dir.y * dir.y;
            double dist = sqrt(dist_squared); 

            // border_to_border distance
            double btb_dist = dist - node_size - grid_node_size;

            
            double rep_force = 0.;
            double attr_force = 0.;
            // overlaping
            if ( btb_dist < 0 ) {

                if ( dist_squared > seuilrep ) {
                    rep_force = repulsion_coeff * 0.3 * (node_degrees[grid_node]+1) * (node_degrees[node_index]+1);
                } else {
                    rep_force = repulsion_coeff / seuilrep;
                }

            } else {
                attr_force = btb_dist;
                rep_force = repulsion_coeff * (node_degrees[grid_node]+1) * (node_degrees[node_index]+1) / btb_dist;
            }

            forces[grid_node][0] = forces[grid_node][0] + dir.x * rep_force - dir.x * attr_force ;
            forces[grid_node][1] = forces[grid_node][1] + dir.y * rep_force - dir.y * attr_force ;
            forces[node_index][0] = forces[node_index][0] - dir.x * rep_force + dir.x * attr_force;
            forces[node_index][1] = forces[node_index][1] - dir.y * rep_force + dir.y * attr_force;
        }

    }

}

void noverlap_force(double (*forces)[2], double FMaxX, double FMaxY) {

    int grid_length = 20;
    int grid_size = 400;

    double center_width = Lx * 0.5;
    double center_height = Ly * 0.5;

    double grid_width = Lx / grid_length;
    double grid_height = Ly / grid_length;

    int* map = (int*) malloc(sizeof(int) * num_nodes);
    for (int i = 0; i < num_nodes; ++i) {
        map[i] = -1;
    }

    if ( modified_graph && mode != 1 ) 
        calculate_node_degrees();

    for (int i = 0; i < num_nodes; ++i) {

        if ( vertices[i].deleted == 0 ) {
            double node_size = initial_node_size + degree_scale_factor * node_degrees[i];
            double minX = vertices[i].x - node_size + center_width / 2 < 0 ? 0 : vertices[i].x - node_size + center_width / 2;
            double maxX = vertices[i].x + node_size + center_width / 2 < 0 ? 0 : vertices[i].x + node_size + center_width / 2;
            double minY = vertices[i].y - node_size + center_height / 2 < 0 ? 0 : vertices[i].y - node_size + center_height / 2;
            double maxY = vertices[i].y + node_size + center_height / 2 < 0 ? 0 : vertices[i].y + node_size + center_height / 2;
            int grid_minX = fmin(fmax(minX / grid_width, 0), grid_length - 1);
            int grid_maxX = fmax(fmin(maxX / grid_width, grid_length - 1), 0);
            int grid_minY = fmin(fmax(minY / grid_height, 0), grid_length - 1);
            int grid_maxY = fmax(fmin(maxY / grid_height, grid_length - 1), 0);

            for (int j = grid_minY; j <= grid_maxY; ++j) {
                for (int k = grid_minX; k <= grid_maxX; ++k) {
                    noverlap_sequential(forces, i, j * grid_length + k, map, FMaxX, FMaxY);
                }
            }
        }

    }

    free(map);
    freeSpatialCell(grid_list, grid_size);
    grid_list = NULL;
}

struct map_arguments {
    int num_clusters;
    double Lx, Ly;
    double ** new_centers;
    int * counts, * labels;
    int vertex_index;

    pthread_mutex_t *lock;
    Barrier barrier;
};

void kmeans_map(void * arg) {

    struct map_arguments * arguments = (struct map_arguments*) arg;
    double min_dist = DBL_MAX;
    int best_cluster = 0;

    for ( int i = 0; i < arguments->num_clusters; ++i ) {
        double dist = squared_distance(centers[i], vertices[arguments->vertex_index]);

        if (dist < min_dist) {
            min_dist = dist;
            best_cluster = i;
        }

    }

    double adjusted_x = vertices[arguments->vertex_index].x;
    double adjusted_y = vertices[arguments->vertex_index].y;

    while (adjusted_x - centers[best_cluster][0] > arguments->Lx / 2) adjusted_x -= arguments->Lx;
    while (centers[best_cluster][0] - adjusted_x > arguments->Lx / 2) adjusted_x += arguments->Lx;
    while (adjusted_y - centers[best_cluster][1] > arguments->Ly / 2) adjusted_y -= arguments->Ly;
    while (centers[best_cluster][1] - adjusted_y > arguments->Ly / 2) adjusted_y += arguments->Ly;

    pthread_mutex_lock(arguments->lock);
    arguments->new_centers[best_cluster][0] += adjusted_x;
    arguments->new_centers[best_cluster][1] += adjusted_y;
    arguments->counts[best_cluster]++;
    pthread_mutex_unlock(arguments->lock);

    arguments->labels[arguments->vertex_index] = best_cluster;

    decrement_barrier(arguments->barrier, 1);
}

// Assigner des noeuds aux clusters et mettre à jour les centres en utilisant l'algorithme k-means
void kmeans_iteration(int num_points, int num_clusters, int *labels, double centers[][2], double Lx, double Ly, double* max_diff) {

    // Modification pour utiliser moins de memoire et eviter un seg fault
    int* counts = (int*) malloc(sizeof(int) * num_clusters);
    double ** new_centers = (double**) malloc(sizeof(double*) * num_clusters);  // Stocker les nouveaux centres calculés

    for (int i = 0; i < num_clusters; ++i) {
        new_centers[i] = (double*) calloc(2, sizeof(double));
    }

    struct barrier bar;
    new_barrier(&bar, num_points);

    pthread_mutex_t lck;
    pthread_mutex_init(&lck, NULL);

    // Assigner chaque point au cluster le plus proche et mettre à jour les centres
    for (int i = 0; i < num_points; i++) {
        if ( vertices[i].deleted == 0 ) {
            struct map_arguments * arguments = (struct map_arguments*) malloc(sizeof(struct map_arguments));
            arguments->Lx = Lx;
            arguments->Ly = Ly;
            arguments->vertex_index = i;
            arguments->counts = counts;
            arguments->new_centers = new_centers;
            arguments->labels = labels;
            arguments->num_clusters = num_clusters;
            arguments->barrier = &bar;
            arguments->lock = &lck;

            struct Job task;
            task.j = kmeans_map;
            task.args = arguments;
            submit(&pool, task);
        } else {
            decrement_barrier(&bar, 1);
        }
    }

    wait_barrier(&bar);   
    pthread_mutex_destroy(&lck);

    // Mise à jour des centres de clusters en fonction des nouvelles assignations
    for (int i = 0; i < num_clusters; i++) {
        if (counts[i] > 0) {
            Point old_center;
            old_center.x = centers[i][0];
            old_center.y = centers[i][1];

            centers[i][0] = new_centers[i][0] / counts[i];
            centers[i][1] = new_centers[i][1] / counts[i];

            double dist = squared_distance(centers[i], old_center);
            *max_diff = *max_diff < dist? dist : *max_diff;

            // Ramener les centres dans l'espace torique
            if ( centers[i][0] < -Lx / 2) centers[i][0] = Lx;
            if ( centers[i][0] > Lx / 2 ) centers[i][0] = Lx;
            if ( centers[i][1] < -Ly / 2) centers[i][1] = Ly;
            if ( centers[i][1] > Ly / 2 ) centers[i][1] = Ly;
        }

        free(new_centers[i]);
    }

    free(new_centers);
    free(counts);
}

struct intra_cluster_args {
    double(*forces)[2];
    int cluster_id;
    double FMaxX, FMaxY;
    Barrier barrier;
};

void repulsion_intra_cluster_job(void * args) {
    struct intra_cluster_args* arguments = (struct intra_cluster_args*) args;
    double (*forces)[2] = arguments->forces;
    double FMaxX = arguments->FMaxX;
    double FMaxY = arguments->FMaxY;
    int cluster_id = arguments->cluster_id;

    int size = cluster_nodes[cluster_id].size;
    for (int i = 0; i < size; ++i) {
        int node_i = cluster_nodes[cluster_id].nodes[i];
    
        Point pi = vertices[node_i];
        if ( pi.deleted == 0 ) {
                
            for (int j = i + 1; j < size; j++) {
                int node_j = cluster_nodes[cluster_id].nodes[j];

                if ( vertices[node_j].deleted == 0 ) {
                    Point dir;
                    toroidal_vector(&dir, pi, vertices[node_j]);
            
                    double dist_squared = dir.x * dir.x + dir.y * dir.y;
                    if (dist_squared > seuilrep) { // Assume a minimum distance to avoid division by zero
                        //double dist = sqrt(dist_squared);

                        double rep_force;
                        if ( mode == 1 ) { 
                            rep_force = repulsion_coeff / (dist_squared*dist_squared);
                        } else if (mode == 2 && communities[i] != communities[j]) {//printf("extra repulsion %d, %d \n",i,j);
                            rep_force = 100000 * repulsion_coeff * (node_degrees[i]+1) * (node_degrees[j]+1) / dist_squared;
                        } else { // mode == 0 est le mode par defaut
                            rep_force = repulsion_coeff * (node_degrees[i]+1) * (node_degrees[j]+1) / dist_squared;
                        } 
                                                
                        forces[node_i][0] -= dir.x * rep_force;
                        forces[node_i][1] -= dir.y * rep_force;
                        forces[node_j][0] += dir.x * rep_force;
                        forces[node_j][1] += dir.y * rep_force;
                    } else {
                        double rep_force = repulsion_coeff / seuilrep;
                                            
                        forces[node_i][0] -= dir.x * rep_force;
                        forces[node_i][1] -= dir.y * rep_force;
                        forces[node_j][0] += dir.x * rep_force;
                        forces[node_j][1] += dir.y * rep_force;
                    }
                }
            }
            
            // capper les forces d'attractions
            forces[node_i][0] = fmax(fmin(forces[node_i][0], FMaxX), -FMaxX);
            forces[node_i][1] = fmax(fmin(forces[node_i][1], FMaxY), -FMaxY);
        
        }
    }
    decrement_barrier(arguments->barrier, 1);
}

void parallel_intra_clusters(double(*forces)[2], double FMaxX, double FMaxY) {
    if ( modified_graph && mode != 1 ) 
        calculate_node_degrees();


    struct barrier bar;
    new_barrier(&bar, n_clusters);
    for (int i = 0; i < n_clusters; ++i) {
        struct intra_cluster_args* arguments = (struct intra_cluster_args*) malloc(sizeof(struct intra_cluster_args));
        arguments->forces = forces;
        arguments->cluster_id = i;
        arguments->FMaxX = FMaxX;
        arguments->FMaxY = FMaxY;
        arguments->barrier = &bar;

        struct Job task;
        task.j = repulsion_intra_cluster_job;
        task.args = arguments;
        submit(&pool, task);
    }
    wait_barrier(&bar);
}

// Étape 2 : Forces de répulsion intra-cluster
void repulsion_intra_clusters(double(*forces)[2], double FMaxX, double FMaxY)
{

    if ( modified_graph && mode != 1 ) 
        calculate_node_degrees();

    for (int cluster = 0; cluster < n_clusters; cluster++) {
        int size = cluster_nodes[cluster].size;
        for (int i = 0; i < size; i++) {
            int node_i = cluster_nodes[cluster].nodes[i];
    
            Point pi = vertices[node_i];
            if ( pi.deleted == 0 ) {
                
                for (int j = i + 1; j < size; j++) {
                    int node_j = cluster_nodes[cluster].nodes[j];

                    if ( vertices[node_j].deleted == 0 ) {
                        Point dir;
                        toroidal_vector(&dir, pi, vertices[node_j]);
            
                        double dist_squared = dir.x * dir.x + dir.y * dir.y;
                        if (dist_squared > seuilrep) { // Assume a minimum distance to avoid division by zero
                            //double dist = sqrt(dist_squared);

                            double rep_force;
                            if ( mode == 1 ) { 
                                rep_force = repulsion_coeff / (dist_squared*dist_squared);
                            } else if (mode == 2 && communities[i] != communities[j]) {//printf("extra repulsion %d, %d \n",i,j);
                                rep_force = 100000 * repulsion_coeff * (node_degrees[i]+1) * (node_degrees[j]+1) / dist_squared;
                            } else { // mode == 0 est le mode par defaut
                                rep_force = repulsion_coeff * (node_degrees[i]+1) * (node_degrees[j]+1) / dist_squared;
                            } 
                                                
                            forces[node_i][0] -= dir.x * rep_force;
                            forces[node_i][1] -= dir.y * rep_force;
                            forces[node_j][0] += dir.x * rep_force;
                            forces[node_j][1] += dir.y * rep_force;
                        } else {
                            double rep_force = repulsion_coeff / seuilrep;
                                                
                            forces[node_i][0] -= dir.x * rep_force;
                            forces[node_i][1] -= dir.y * rep_force;
                            forces[node_j][0] += dir.x * rep_force;
                            forces[node_j][1] += dir.y * rep_force;
                        }
                    }
                }
            
                // capper les forces d'attractions
                forces[node_i][0] = fmax(fmin(forces[node_i][0], FMaxX), -FMaxX);
                forces[node_i][1] = fmax(fmin(forces[node_i][1], FMaxY), -FMaxY);
        
            }
        }
    }


}

// etape 4 dans update_positions
void update_clusters()
{

    int modified = 0;
    if (kmeans_mode != 0 && iteration % (saut * (1 + espacement)) == 0) {
        int centers_converged = 0;
        while (centers_converged == 0) {
            // the biggest difference between the former centers and the new ones
            double max_movement = 0;

            kmeans_iteration(num_nodes, n_clusters, clusters, centers, Lx, Ly, &max_movement);

            centers_converged = max_movement <= epsilon;
        }
        modified = 1;
    } else if ( kmeans_mode == 0 ) {
        grid_clustering(num_nodes, n_clusters, clusters, Lx, Ly);
        modified = 1;
    }

    if ( modified ) {
        clear_clusters();
        for (int i = 0; i < num_nodes; i++) {
            int best_cluster = clusters[i];
            if ( vertices[i].deleted == 0 ) {
                add_node_to_cluster(best_cluster, i);
            }
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
    int estimated_capacity = (live_nodes / num_clusters) + 1;
    cluster_nodes = (Cluster*) malloc(num_clusters * sizeof(Cluster));
    for (int i = 0; i < num_clusters; i++) {
        cluster_nodes[i].nodes = (int*) malloc(estimated_capacity * sizeof(int));
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