#include <math.h>
#include <string.h>
#include <time.h>
#include <float.h>
#include <stdbool.h>
#include <termios.h>
#include <ctype.h>

#include <stdatomic.h>

#include "global.h"

#ifdef _DEBUG_
    #include "debug/debug_time.h"
#endif

#include "../../out/graph_Graph.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../../lib/stb_image_write.h"


int modeA=0; //mode pour afficher des noeuds en fonction de classe

int iteration = 0;
double Max_movementOld = 0;
int max_iterations = 5000;

short pause_updates = 0;


/**
 * 
 * 
 * 
 Fonction Pour L'interface
 * 
 * 
 * 
 * 
 */

JNIEXPORT jboolean JNICALL Java_graph_Graph_updatePositions
 (JNIEnv * env, jobject obj)
{
   double Max_movementOld = 0.0;
   double Max_movement;
   double FMaxX = Lx/(friction*1000);
   double FMaxY = Ly/(friction*1000);
   thresholdS = (Lx/4000)*(Ly/4000);
   thresholdA = (Lx/4000)*(Ly/4000);
   epsilon = (Lx/800)*(Ly/800);
   seuilrep = (Lx/1000)*(Lx/1000);

   double PasMaxX = Lx / 10.;
   double PasMaxY = Ly / 10.;

   iteration = 0;
   static Point forces[MAX_NODES] = {0};

   if ( pause_updates == 0 ){
        repulsion_edges(forces);
        repulsion_intra_clusters(forces, FMaxX, FMaxY);
        repulsion_anti_edges(forces);
        Max_movement = update_position_forces(forces, PasMaxX, PasMaxY, Max_movement);
        update_clusters();

        ++iteration;

        if (Max_movement==Max_movementOld) {
            friction *= 0.7;
        }
           
        Max_movementOld=Max_movement;
        friction *= amortissement;
   }

   if ((Max_movement < thresholdS && iteration > 50) || iteration > max_iterations-1 ) {
        // end state reached
        pause_updates = 1;

        return 0;
   }

   return 1;
}

JNIEXPORT jintArray JNICALL Java_graph_Graph_getCommunities
  (JNIEnv * env, jobject obj)
{
    jintArray result = (*env)->NewIntArray(env, MAX_NODES);

    (*env)->SetIntArrayRegion(env, result, 0, MAX_NODES, communities);

    return result;
}

JNIEXPORT jobjectArray JNICALL Java_graph_Graph_getClusterColors
  (JNIEnv * env, jobject obj)
{
    jclass obj_class = (*env)->FindClass(env, "[F");
    jobjectArray result = (*env)->NewObjectArray(env, MAX_NODES, obj_class, 0);

    for (int i = 0; i < MAX_NODES; ++i)
    {
        jfloatArray float_array = (*env)->NewFloatArray(env, 3);
        (*env)->SetFloatArrayRegion(env, float_array, 0, 3, cluster_colors[i]);

        (*env)->SetObjectArrayElement(env, result, i, float_array);
    }

    return result;
}

JNIEXPORT jobjectArray JNICALL Java_graph_Graph_getEdges
  (JNIEnv * env, jobject obj)
{
    // remplacer "backendinterface/Edge" par "[packageName]/[nomClasse]"
    jclass obj_class = (*env)->FindClass(env, "graph/EdgeC");
    jmethodID edge_constructor = (*env)->GetMethodID(env, obj_class, "<init>", "(IID)V");
    jobject initial_elem = (*env)->NewObject(env, obj_class, edge_constructor, 0, 0, 0.);
    
    jobjectArray result = (*env)->NewObjectArray(env, num_edges, obj_class, initial_elem);

    for (int i = 0; i < num_edges; ++i)
    {
        int node1 = edges[i].node1;
        int node2 = edges[i].node2;
        double weight = edges[i].weight;
        jobject edge = (*env)->NewObject(env, obj_class, edge_constructor, node1, node2, weight);
    
        (*env)->SetObjectArrayElement(env, result, i, edge);
    }

    return result;
}

JNIEXPORT jobjectArray JNICALL Java_graph_Graph_getPositions
  (JNIEnv * env, jobject obj)
{
  jclass obj_class = (*env)->FindClass(env, "graph/Vertex");
  jmethodID point_constructor = (*env)->GetMethodID(env, obj_class, "<init>", "(DD)V");

  jobject initial_elem = (*env)->NewObject(env, obj_class, point_constructor, 0., 0.);

  jobjectArray result = (*env)->NewObjectArray(env, num_nodes, obj_class, initial_elem);

  for (int i = 0; i < num_nodes; ++i) {
    double x = vertices[i].x;
    double y = vertices[i].y;

    jobject point = (*env)->NewObject(env, obj_class, point_constructor, x, y);

    (*env)->SetObjectArrayElement(env, result, i, point);
  }

  return result;
}

JNIEXPORT jobject JNICALL Java_graph_Graph_startsProgram
  (JNIEnv * env, jobject obj, jstring filepath)
{
    srand(time(NULL));

    jboolean b = JNI_FALSE;
    const char* str = (*env)->GetStringUTFChars(env, filepath, &b);

    load_csv_data(str);

    jclass obj_class = (*env)->FindClass(env, "[D");
    jobjectArray result = (*env)->NewObjectArray(env, num_rows, obj_class, 0);

    for (int i = 0; i < num_rows; ++i)
    {
        jdoubleArray double_array = (*env)->NewDoubleArray(env, num_columns);
        (*env)->SetDoubleArrayRegion(env, double_array, 0, num_columns, data[i]);

        (*env)->SetObjectArrayElement(env, result, i, double_array);
    }

    (*env)->ReleaseStringUTFChars(env, filepath, str); 

    return result;

}

JNIEXPORT jobject JNICALL Java_graph_Graph_computeThreshold
  (JNIEnv * env, jobject obj, jint modeSimilitude)
{

    InitPool(&pool, 1000, 8);

    num_nodes = num_rows;

    double threshold, antiseuil;
    mode_similitude = modeSimilitude;

    // un tableau dont de taille nombre de paire dans un ensemble avec num_rows element = 2 parmis num_rows
    double * similarities = (double*) malloc(num_rows * (num_rows - 1) / 2 * sizeof(double));
    #ifdef _DEBUG_
        printf("debut log");
        struct chrono chr;
        chr_assign_log(&chr, "similitude.csv");
        chr_start_clock(&chr);
        double means_similitude = calculate_mean_similitude_parallel(modeSimilitude, similarities);
        chr_stop(&chr);
        chr_close_log(&chr);
        printf("fin log");
    #else
        double means_similitude = calculate_mean_similitude_parallel(modeSimilitude, similarities);
    #endif

    calculate_threshold(modeSimilitude, 10*num_nodes, &threshold, &antiseuil, similarities);

    printf("Seuil recommandé: %lf, %lf\n", threshold, antiseuil);
    printf("%lf, %d", means_similitude, num_rows);

    jclass res_class = (*env)->FindClass(env, "graph/Metadata");
    jmethodID constructor = (*env)->GetMethodID(env, res_class, "<init>", "(IDDD)V");
    jobject res = (*env)->NewObject(env, res_class, constructor, num_nodes, threshold, antiseuil, means_similitude);

    free(similarities);

    return res;
}


JNIEXPORT jobject JNICALL Java_graph_Graph_initiliazeGraph
  (JNIEnv *env, jobject obj, jint md, jdouble thresh, jdouble anti_thresh)
{

    #ifdef _DEBUG_
        printf("debut log");
        struct chrono chr;
        chr_assign_log(&chr, "edges.csv");
        chr_start_clock(&chr);
        calculate_similitude_and_edges(mode_similitude, thresh, anti_thresh);
        chr_stop(&chr);
        chr_close_log(&chr);
        printf("fin log");
    #else
        calculate_similitude_and_edges(mode_similitude, thresh, anti_thresh);
    #endif

    modeA=0;
    if (md == 0) {
        num_communities = louvain_method();
    } else if (md == 1) {
        num_communities = louvain_methodC();
    } else if (md == 2) {
        num_communities = leiden_method();
    } else if (md == 3) {
        num_communities = leiden_method_CPM();
    } else if (md == 4) {
        // TODO 
        //int nbValeurs;
        //int S[MAX_NODES]={0};
        num_communities = leiden_method_CPM();
        // Demander le chemin du fichier à l'utilisateur
        lireColonneCSV(S, &nbValeurs);
        // Afficher les valeurs lues
        printf("nombres de valeurs lues : %d pour %d données\n",nbValeurs,num_nodes);
        modeA =1;
        compute_ratio_S(S);
    } else {
        printf("Option invalide\n");
    }
    initialize_community_colors();

    #ifdef _DEBUG_
        printf("Fin Calcul Communaute");
    #endif

    compute_average_vectors();

    #ifdef _DEBUG_
        printf("Vecteurs moyens fini \n");
    #endif

    for (int i = 0; i < num_nodes; i++) {
        random_point_in_center(i);
        velocities[i].x = velocities[i].y = 0.0;
    }
    n_clusters = (int)sqrt(num_nodes);
    init_clusters(n_clusters);
    initialize_centers();
    assign_cluster_colors();
    calculate_node_degrees();

    jclass res_class = (*env)->FindClass(env, "graph/Metadata");
    jmethodID constructor = (*env)->GetMethodID(env, res_class, "<init>", "(IDDIII)V");
    jobject res = (*env)->NewObject(env, res_class, constructor, num_nodes, thresh, anti_thresh, num_edges, num_antiedges, n_clusters);

    return res;   

}

JNIEXPORT void JNICALL Java_graph_Graph_freeAllocatedMemory
  (JNIEnv * env, jobject obj)
{

    // Libérer la mémoire allouée pour les voisins
    for (int i = 0; i < num_nodes; i++) {
    	Neighbor* neighbor = adjacency_list[i].head;
        while (neighbor != NULL) {
           Neighbor* next = neighbor->next;
           free(neighbor);
           neighbor = next;
        }
    }
    free_clusters();

    FreePool(&pool);
}

JNIEXPORT void JNICALL Java_graph_Graph_setSaut
  (JNIEnv * env, jobject obj, jint s)
{
    saut = s;
    espacement = 1;
}

JNIEXPORT void JNICALL Java_graph_Graph_setThresholdS
  (JNIEnv * env, jobject obj, jdouble thresh)
{
    thresholdS = thresh;
}

JNIEXPORT void JNICALL Java_graph_Graph_setFriction
  (JNIEnv * env, jobject obj, jdouble f)
{
    friction = f;
}

JNIEXPORT void JNICALL Java_graph_Graph_setModeRepulsion
  (JNIEnv * env, jobject obj, jint modeRepulsion)
{
    mode = modeRepulsion;
}

JNIEXPORT void JNICALL Java_graph_Graph_setAntiRepulsion
  (JNIEnv * env, jobject obj, jdouble repulsion)
{
    coeff_antiarete = repulsion;
}

JNIEXPORT void JNICALL Java_graph_Graph_setAttractionCoeff
  (JNIEnv * env, jobject obj, jdouble coeff)
{
    attraction_coeff = coeff;
}

JNIEXPORT void JNICALL Java_graph_Graph_setThresholdA
  (JNIEnv * env, jobject obj, jdouble thresh)
{
    thresholdA = thresh;
}

JNIEXPORT void JNICALL Java_graph_Graph_setSeuilRep
  (JNIEnv * env, jobject obj, jdouble seuil)
{
    seuilrep = seuil;
}

JNIEXPORT void JNICALL Java_graph_Graph_setDimension
  (JNIEnv * env, jobject obj, jdouble width, jdouble height)
{
    Lx = width;
    Ly = height;
}

JNIEXPORT void JNICALL Java_graph_Graph_setAmortissement
  (JNIEnv * env, jobject obj, jdouble amort)
{
    amortissement = amort;
}

JNIEXPORT void JNICALL Java_graph_Graph_setNodePosition
  (JNIEnv * env, jobject obj, jint index, jdouble x, jdouble y)
{
    vertices[index].x = x;
    vertices[index].y = y;
}

JNIEXPORT void JNICALL Java_graph_Graph_unpauseGraph
  (JNIEnv * env, jobject obj)
{
    if ( pause_updates == 1 ) {
        iteration = 0;
        pause_updates = 0;
    }
}

JNIEXPORT void JNICALL Java_graph_Graph_SetNumberClusters
  (JNIEnv * env, jobject obj, jint new_n_clusters)
{
    free_clusters();
    n_clusters = new_n_clusters;

    init_clusters(n_clusters);
    initialize_centers();
    assign_cluster_colors();
}
