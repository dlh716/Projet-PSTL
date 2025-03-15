#ifndef FORCEATLAS_GRAPH_H
#define FORCEATLAS_GRAPH_H

#include <math.h>
#include <stddef.h>

#define MAX_NODES 50000
#define MAX_EDGES 1000000

// Structures de données pour représenter les arêtes, points, et clusters
typedef struct {
    int node1;
    int node2;
    double weight;
} Edge;

typedef struct {
    double x;
    double y;
} Point;

extern Edge edges[MAX_EDGES]; // Pour les arêtes normales
extern char *node_names[MAX_NODES]; // Array to store node names as strings      
extern int S[MAX_NODES];
extern Point positions[MAX_NODES];
extern Point velocities[MAX_NODES];
extern int node_degrees[MAX_NODES];

extern _Atomic int num_edges;
extern _Atomic int num_antiedges;
extern Edge antiedges[MAX_EDGES];  // Pour les anti-arêtes

extern int num_nodes;
extern double Lx, Ly;

extern int iteration;

extern double friction;
// TODO passer en argument a calculate_similitude dans la version de base
extern double coeff_antiarete; // Facteur de répulsion des antiarêtes
extern double attraction_coeff;
extern double thresholdA;
extern double seuilrep;
extern double thresholdS;

// Calculer un vecteur avec un enroulement toroïdal
void toroidal_vector(Point *dir, Point p1, Point p2);

void repulsion_edges(Point* forces);
void repulsion_anti_edges(Point* forces);
double update_position_forces(Point* forces, double PasMaxX, double PasMaxY, double Max_movement);


#endif