#ifndef FORCEATLAS_GLOBAL_H
#define FORCEATLAS_GLOBAL_H

#include <jni.h>
#include "c_graph/graph.h"

#define MAX_NODES 100000
#define MAX_EDGES 1000000

extern Edge edges[MAX_EDGES]; // Pour les arêtes normales
extern char *node_names[MAX_NODES]; // Array to store node names as strings      
extern int S[MAX_NODES];
extern Point vertices[MAX_NODES];
extern Point velocities[MAX_NODES];
extern int node_degrees[MAX_NODES];

extern _Atomic int num_edges;
extern _Atomic int num_antiedges;
extern Edge antiedges[MAX_EDGES];  // Pour les anti-arêtes

extern int num_nodes, live_nodes;
extern double Lx, Ly;
extern short modified_graph;

extern double friction;
// TODO passer en argument a calculate_similitude dans la version de base
extern double coeff_antiarete; // Facteur de répulsion des antiarêtes
extern double attraction_coeff;
extern double thresholdA;
extern double seuilrep;
extern double thresholdS;
extern double amortissement;

#include "c_graph/cluster.h"

// Structures utilisées dans la méthode de Louvain
extern jint communities[MAX_NODES]; // Stocke les communautés détectées par Louvain
extern int clusters[MAX_NODES];
extern float cluster_colors[MAX_NODES][3];
extern double centers[MAX_NODES][2];
extern Cluster *cluster_nodes;  // Tableau de clusters
extern int n_clusters;
extern int kmeans_mode;

extern double epsilon;

extern int espacement;

// modifiable par utilisateur
extern double repulsion_coeff;
extern int saut;
extern int mode;

#include "c_graph/communities.h"

extern AdjacencyList adjacency_list[MAX_NODES];
extern Community node_community_map[MAX_NODES];
extern int num_communities;
extern int component_sizes[MAX_NODES]; // Tableau pour stocker la taille de chaque composante
extern int num_components;
extern double lambda;

#include "concurrent/Barrier.h"

#include "concurrent/Pool.h"

extern struct Pool pool;

#include "concurrent/Tools.h"

#ifndef _DEBUG_
    #include "debug/debug_time.h"
#endif

#include "pretraitement/data.h"

#define MAX_LINE_LENGTH 10000  // Longueur maximale d'une ligne dans le fichier

// TODO On peut peut-etre s'en passer
extern int nbValeurs;
extern double **data;  // Stocker les données CSV
extern int num_rows, num_columns;
extern char delimiter[1];
extern int S[MAX_NODES];


#include "pretraitement/similarity.h"

#define NUM_BINS 100  // Nombre de bins pour l'histogramme
#define EPSILON 1e-12  // Pour éviter la division par 0

extern int mode_similitude;
extern double **similarity_matrix;
extern int global_histogram[NUM_BINS];

// ForceAtlas

extern int iteration;

#endif