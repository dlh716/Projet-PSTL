#include "graph.h"
#include "../global.h"

Point vertices[MAX_NODES];

Edge edges[MAX_EDGES]; // Pour les arêtes normales   
double velocities[MAX_NODES][2];
int node_degrees[MAX_NODES];

_Atomic int num_edges = 0;
_Atomic int num_antiedges = 0;
double coeff_antiarete = 100; // Facteur de répulsion des antiarêtes
Edge antiedges[MAX_EDGES];  // Pour les anti-arêtes

int num_nodes = 0;
int live_nodes = 0;
double Lx = 300, Ly = 300;

double friction = 0.1;
double attraction_coeff = 10;
double thresholdA = 1;
double seuilrep = 0;
double thresholdS = 1;

double amortissement = 0.999;

double** similarity_matrix = NULL;

short modified_graph = 0;

void toroidal_vector(Point *dir, Point p1, Point p2) {
    dir->x = p2.x - p1.x;
    dir->y = p2.y - p1.y;
    if (fabs(dir->x) > Lx / 2) dir->x -= copysign(Lx, dir->x);
    if (fabs(dir->y) > Ly / 2) dir->y -= copysign(Ly, dir->y);
}

// Calculer la distance toroïdale entre deux points
double toroidal_distance(Point p1, Point p2) {
    Point dir;
    toroidal_vector(&dir, p1, p2);
    return sqrt(dir.x * dir.x + dir.y * dir.y);
}

// Calculer les degrés de chaque noeud
// Works with dot files
void calculate_node_degrees(void) {
    for (int i = 0; i < num_nodes; i++) {
        node_degrees[i] = 0;
    }

    for (int i = 0; i < num_edges; i++) {
        int node1 = edges[i].node1;
        int node2 = edges[i].node2;

        if ( vertices[node1].deleted == 0 && vertices[node2].deleted == 0) {
            node_degrees[node1]++;
            node_degrees[node2]++;
        }
    }

    modified_graph = 0;

}

// Générer un point aléatoire près du centre
void random_point_in_center(int index) {
    double center_width = Lx * 0.3;
    double center_height = Ly * 0.3;
    vertices[index].x = (rand() / (double)RAND_MAX) * center_width - center_width / 2;
    vertices[index].y = (rand() / (double)RAND_MAX) * center_height - center_height / 2;
    vertices[index].deleted = 0;
}

void translate_positions(double dx, double dy) {
    double half_Lx = Lx / 2.0;
    double half_Ly = Ly / 2.0;
    for (int i = 0; i < num_nodes; ++i) {
        double x = vertices[i].x + dx;
        double y = vertices[i].y + dy;
        // Appliquer les conditions aux limites toroïdales

        while ( x < -half_Lx) { x += Lx; }
        while ( x > half_Lx)  { x -= Lx; }
        while ( y < -half_Ly) { y += Ly; }
        while ( y > half_Ly)  { y -= Ly; }

        vertices[i].x = x;
        vertices[i].y = y;
    }
}

struct repulsion_args {
    int start, end;
    double(*forces)[2];
    pthread_mutex_t* forces_lock;
    Barrier barrier;
};

void repulsionTask_Edge(void* args) {

    struct repulsion_args* arguments = (struct repulsion_args*) args;

    for (int edge_index = arguments->start; edge_index < arguments->end; edge_index++) {
        int node1 = edges[edge_index].node1;
        int node2 = edges[edge_index].node2;
    
        if ( vertices[node1].deleted == 0 && vertices[node2].deleted == 0 ) {
            Point dir;
            toroidal_vector(&dir, vertices[node1], vertices[node2]);
        
            double dist_squared = dir.x * dir.x + dir.y * dir.y;
            double att_force = attraction_coeff; //*dist_squared;
            
            if (dist_squared > thresholdA) {
                pthread_mutex_lock(arguments->forces_lock);     
                arguments->forces[node1][0] += dir.x * att_force;
                arguments->forces[node1][1] += dir.y * att_force;
                arguments->forces[node2][0] -= dir.x * att_force;
                arguments->forces[node2][1] -= dir.y * att_force;
                pthread_mutex_unlock(arguments->forces_lock);  
            }
        }
    }

    decrement_barrier(arguments->barrier, 1);
}

void repulsionTask_AntiEdge(void *args) {

    struct repulsion_args* arguments = (struct repulsion_args*) args;

    for (int edge_index = arguments->start; edge_index < arguments->end; edge_index++) {
        int node1 = antiedges[edge_index].node1;
        int node2 = antiedges[edge_index].node2;
    
        if ( vertices[node1].deleted == 0 && vertices[node2].deleted == 0 ) {
            Point dir;
            toroidal_vector(&dir, vertices[node1], vertices[node2]);
        
            double dist = sqrt(dir.x * dir.x + dir.y * dir.y);
            if (dist > seuilrep) {
                double rep_force = coeff_antiarete/(dist*dist);

                pthread_mutex_lock(arguments->forces_lock);
                arguments->forces[node1][0] -= (dir.x / dist) * rep_force;
                arguments->forces[node1][1] -= (dir.y / dist) * rep_force;
                arguments->forces[node2][0] += (dir.x / dist) * rep_force;
                arguments->forces[node2][1] += (dir.y / dist) * rep_force;
                pthread_mutex_unlock(arguments->forces_lock);
            } else {
                double rep_force = coeff_antiarete/ seuilrep;
                     
                pthread_mutex_lock(arguments->forces_lock);
                arguments->forces[node1][0] -= dir.x * rep_force;
                arguments->forces[node1][1] -= dir.y * rep_force;
                arguments->forces[node2][0] += dir.x * rep_force;
                arguments->forces[node2][1] += dir.y * rep_force;
                pthread_mutex_unlock(arguments->forces_lock);
            }
        }
    }

    decrement_barrier(arguments->barrier, 1);
}

void submit_repulsionTask(int start, int end, double(*forces)[2], Barrier b, pthread_mutex_t* lk, job j) {
    struct repulsion_args* args = (struct repulsion_args*) malloc(sizeof(struct repulsion_args));
    args->start = start;
    args->end = end;
    args->forces = forces;
    args->barrier = b;
    args->forces_lock = lk;

    struct Job task;
    task.j = j;
    task.args = args;
    submit(&pool, task);
}

void parallel_repulsion(double(*forces)[2], job j) {
    struct barrier b;
    new_barrier(&b, pool.nb_threads);

    pthread_mutex_t lk;
    pthread_mutex_init(&lk, NULL);

    int edge_chunck = num_edges / pool.nb_threads;

    for (int i = 0; i < pool.nb_threads - 1; ++i)
    {
        submit_repulsionTask(i * edge_chunck, (i+1) * edge_chunck, forces, &b, &lk, j);
    }

    submit_repulsionTask((pool.nb_threads-1) * edge_chunck, num_edges, forces, &b, &lk, j);

    wait_barrier(&b);
    pthread_mutex_destroy(&lk);
}

// probablement privé utilisée dans update_positions
// Étape 1 : Forces d'attraction basées sur les arêtes
void repulsion_edges(double(*forces)[2])
{

    for (int edge_index = 0; edge_index < num_edges; edge_index++) {
        int node1 = edges[edge_index].node1;
        int node2 = edges[edge_index].node2;
    
        if ( vertices[node1].deleted == 0 && vertices[node2].deleted == 0 ) {
            Point dir;
            toroidal_vector(&dir, vertices[node1], vertices[node2]);
        
            double dist_squared = dir.x * dir.x + dir.y * dir.y;
            double att_force = attraction_coeff; //*dist_squared;
            
            if (dist_squared > thresholdA) {            
                forces[node1][0] += dir.x * att_force;
                forces[node1][1] += dir.y * att_force;
                forces[node2][0] -= dir.x * att_force;
                forces[node2][1] -= dir.y * att_force;
            }
        }
    }

}

void parallel_repulsion_edges(double(*forces)[2]) {
    parallel_repulsion(forces, repulsionTask_Edge);
}

void parallel_repulsion_anti_edges(double(*forces)[2]) {
    parallel_repulsion(forces, repulsionTask_AntiEdge);
}

// probablement privé utilisé dans update_positions
// Étape 2 bis : Mettre à jour les positions en fonction des forces
void repulsion_anti_edges(double(*forces)[2])
{

    for (int edge_index = 0; edge_index < num_antiedges; edge_index++) {
        int node1 = antiedges[edge_index].node1;
        int node2 = antiedges[edge_index].node2;
    
        if ( vertices[node1].deleted == 0 && vertices[node2].deleted == 0 ) {
            Point dir;
            toroidal_vector(&dir, vertices[node1], vertices[node2]);
        
            double dist = sqrt(dir.x * dir.x + dir.y * dir.y);
            if (dist > seuilrep) {
                double rep_force = coeff_antiarete/(dist*dist);
                forces[node1][0] -= (dir.x / dist) * rep_force;
                forces[node1][1] -= (dir.y / dist) * rep_force;
                forces[node2][0] += (dir.x / dist) * rep_force;
                forces[node2][1] += (dir.y / dist) * rep_force;
            } else {
                double rep_force = coeff_antiarete/ seuilrep;
                                            
                forces[node1][0] -= dir.x * rep_force;
                forces[node1][1] -= dir.y * rep_force;
                forces[node2][0] += dir.x * rep_force;
                forces[node2][1] += dir.y * rep_force;
            }
        }
    }

}

// probablement privé utilisé dans update_positions
// Étape 3 : Mettre à jour les positions des sommets du graphe en fonction des forces
double update_position_forces(double(*forces)[2], double PasMaxX, double PasMaxY, double Max_movement)
{
    double half_Lx = Lx / 2.0;
    double half_Ly = Ly / 2.0;

    double new_max_movement = 0.0;
    for (int i = 0; i < num_nodes; i++) {
        if ( vertices[i].deleted == 0 ) {
            velocities[i][0] = (velocities[i][0] + forces[i][0]) * friction;
            velocities[i][1] = (velocities[i][1] + forces[i][1]) * friction;
            velocities[i][0] = fmin(fmax(velocities[i][0], -PasMaxX), PasMaxX); // Capper la force en x à 1
            velocities[i][1] = fmin(fmax(velocities[i][1], -PasMaxY), PasMaxY); // Capper la force en y à 1

            double x = vertices[i].x + velocities[i][0];
            double y = vertices[i].y + velocities[i][1];
            // Appliquer les conditions aux limites toroïdales

            if ( x < -half_Lx) { x = -half_Lx; }
            if ( x > half_Lx)  { x = half_Lx; }
            if ( y < -half_Ly) { y = -half_Ly; }
            if ( y > half_Ly)  { y = half_Ly; }

            vertices[i].x = x;
            vertices[i].y = y;

            new_max_movement = fmax(Max_movement, velocities[i][0] * velocities[i][0] + velocities[i][0] * velocities[i][0]);
        }
    }

    return new_max_movement;
}

// Normaliser un point
void normalize(Point *p) {
    double norm = sqrt(p->x * p->x + p->y * p->y);
    if (norm > 0) {
        p->x /= norm;
        p->y /= norm;
    }
}



struct similarity_args {
    double threshold;
    double antiseuil;
    int row;
    int choice;
    Barrier barrier;
};

void similarity_job(void *args) {
    struct similarity_args *data = (struct similarity_args*) args;

    for (int j = data->row + 1; j < num_rows; ++j) {
        double similarity = similarity_matrix[data->row][j]; 

        if (similarity > data->threshold && num_edges < MAX_EDGES) {
            int edge_index = incr_or_max(&num_edges, MAX_EDGES);
            if (edge_index < MAX_EDGES) {
                edges[edge_index].node1 = data->row;
                edges[edge_index].node2 = j;
                edges[edge_index].weight = similarity;
            }
        } else if (similarity < data->antiseuil && num_antiedges < MAX_EDGES) {
            int antiedge_index = incr_or_max(&num_antiedges, MAX_EDGES);
            if (antiedge_index < MAX_EDGES) {
                antiedges[antiedge_index].node1 = data->row;
                antiedges[antiedge_index].node2 = j;
                antiedges[antiedge_index].weight = similarity;
            }
        }
    }

    decrement_barrier(data->barrier, 1);
}

/////////////////////////////////////


// Fonction pour calculer la similitude en fonction du choix de l'utilisateur
void calculate_similitude_and_edges(int md, double threshold, double antiseuil) {
        
    num_edges = 0;
    num_antiedges = 0;
    struct barrier bar;
    new_barrier(&bar, num_rows);

    for (int i = 0; i < num_rows; i++) {
        struct similarity_args* s_args = (struct similarity_args*) malloc(sizeof(struct similarity_args));
        s_args->threshold = threshold;
        s_args->antiseuil = antiseuil;
        s_args->row       = i;
        s_args->choice    = md;
        s_args->barrier   = &bar;

        struct Job task;
        task.j = similarity_job;
        task.args = s_args;

        submit(&pool, task);
    }
    // main thread wait for the thread to finish processing the previous calculation
    wait_barrier(&bar);

    num_antiedges = fmin(num_antiedges, MAX_EDGES);
    num_edges = fmin(num_edges, MAX_EDGES);
}