#include "graph.h"

Edge edges[MAX_EDGES]; // Pour les arêtes normales
char *node_names[MAX_NODES]; // Array to store node names as strings      
int S[MAX_NODES]={0};
Point positions[MAX_NODES];
Point velocities[MAX_NODES];
int node_degrees[MAX_NODES];

_Atomic int num_edges = 0;
_Atomic int num_antiedges = 0;
double coeff_antiarete = 100; // Facteur de répulsion des antiarêtes
Edge antiedges[MAX_EDGES];  // Pour les anti-arêtes

int num_nodes = 0;
double Lx = 300, Ly = 300;

int iteration=0;

double friction = 0.1;
double attraction_coeff = 100;
double thresholdA = 1;
double seuilrep = 0;
double thresholdS = 1;






void toroidal_vector(Point *dir, Point p1, Point p2) {
    dir->x = p2.x - p1.x;
    dir->y = p2.y - p1.y;
    if (fabs(dir->x) > Lx / 2) dir->x -= copysign(Lx, dir->x);
    if (fabs(dir->y) > Ly / 2) dir->y -= copysign(Ly, dir->y);
}

// probablement privé utilisée dans update_positions
// Étape 1 : Forces d'attraction basées sur les arêtes
void repulsion_edges(Point* forces)
{

    for (int edge_index = 0; edge_index < num_edges; edge_index++) {
        int node1 = edges[edge_index].node1;
        int node2 = edges[edge_index].node2;
    
        Point dir;
        toroidal_vector(&dir, positions[node1], positions[node2]);
    
        double dist_squared = dir.x * dir.x + dir.y * dir.y;
        double att_force = attraction_coeff; //*dist_squared;
        
        if (dist_squared > thresholdA) {            
            forces[node1].x += dir.x * att_force;
            forces[node1].y += dir.y * att_force;
            forces[node2].x -= dir.x * att_force;
            forces[node2].y -= dir.y * att_force;
        }
    }

}

// probablement privé utilisé dans update_positions
// Étape 2 bis : Mettre à jour les positions en fonction des forces
void repulsion_anti_edges(Point* forces)
{

    for (int edge_index = 0; edge_index < num_antiedges; edge_index++) {
        int node1 = antiedges[edge_index].node1;
        int node2 = antiedges[edge_index].node2;
    
        Point dir;
        toroidal_vector(&dir, positions[node1], positions[node2]);
    
        double dist = sqrt(dir.x * dir.x + dir.y * dir.y);
        if (dist > seuilrep) {
            double rep_force = coeff_antiarete/(dist*dist);
            forces[node1].x -= (dir.x / dist) * rep_force;
            forces[node1].y -= (dir.y / dist) * rep_force;
            forces[node2].x += (dir.x / dist) * rep_force;
            forces[node2].y += (dir.y / dist) * rep_force;
        } else {
            double rep_force = coeff_antiarete/ seuilrep;
                                        
            forces[node1].x -= dir.x * rep_force;
            forces[node1].y -= dir.y * rep_force;
            forces[node2].x += dir.x * rep_force;
            forces[node2].y += dir.y * rep_force;
        }
    }

}

// probablement privé utilisé dans update_positions
// Étape 3 : Mettre à jour les positions en fonction des forces
double update_position_forces(Point* forces, double PasMaxX, double PasMaxY, double Max_movement)
{
    double half_Lx = Lx / 2.0;
    double half_Ly = Ly / 2.0;

    double new_max_movement = 0.0;
    for (int i = 0; i < num_nodes; i++) {
        velocities[i].x = (velocities[i].x + forces[i].x) * friction;
        velocities[i].y = (velocities[i].y + forces[i].y) * friction;
        velocities[i].x = fmin(fmax(velocities[i].x, -PasMaxX), PasMaxX); // Capper la force en x à 1
        velocities[i].y = fmin(fmax(velocities[i].y, -PasMaxY), PasMaxY); // Capper la force en y à 1

        positions[i].x += velocities[i].x;
        positions[i].y += velocities[i].y;
                    // Appliquer les conditions aux limites toroïdales
                while (positions[i].x < -half_Lx) positions[i].x += Lx;
                while (positions[i].x > half_Lx) positions[i].x -= Lx;
                while (positions[i].y < -half_Ly) positions[i].y += Ly;
                while (positions[i].y > half_Ly) positions[i].y -= Ly;

        new_max_movement = fmax(Max_movement, velocities[i].x * velocities[i].x + velocities[i].y * velocities[i].y);
    }

    return new_max_movement;
}

