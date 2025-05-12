#include <math.h>
#include <string.h>
#include <time.h>
#include <float.h>
#include <stdbool.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <termios.h>
#endif

#include <ctype.h>

#include <stdatomic.h>

#include "global.h"

#ifdef _DEBUG_
    #include "debug/debug_time.h"
#endif

/* #include "../out/linux/com_mongraphe_graphui_Graph.h" */


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
  double FMaxX = Lx/(friction*1000);
  double FMaxY = Ly/(friction*1000);

  double PasMaxX = Lx / 10.;
  double PasMaxY = Ly / 10.;

  static double forces[MAX_NODES][2] = { {0, 0} };

  if ( pause_updates == 0 ){
       
    repulsion_edges(forces);
    parallel_intra_clusters(forces, FMaxX, FMaxY);
    repulsion_anti_edges(forces);

    double Max_movement = update_position_forces(forces, PasMaxX, PasMaxY, 0.);
    update_clusters();

    ++iteration;

    if (Max_movement == Max_movementOld) {
        friction *= 0.7;
    }
          
    Max_movementOld = Max_movement;
    friction *= amortissement;
  }

  if ((Max_movementOld < thresholdS && iteration > 50) || iteration >= max_iterations ) {
    // end state reached
    pause_updates = 1;
    return 0;
  }

  for(int i = 0; i < num_nodes; ++i) {
   forces[i][0] = 0.;
   forces[i][1] = 0.;
  }

  return 1;
}

JNIEXPORT jintArray JNICALL Java_graph_Graph_getCommunities(JNIEnv *env, jobject obj) {
    jintArray result = (*env)->NewIntArray(env, MAX_NODES);
    (*env)->SetIntArrayRegion(env, result, 0, MAX_NODES,(const jint *) communities);

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
    jclass obj_class = (*env)->FindClass(env, "Lgraph/EdgeC;");
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
  jclass obj_class = (*env)->FindClass(env, "Lgraph/Vertex;");
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
    
    num_rows = 0;

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
  (JNIEnv * env, jobject obj, jint modeSimilitude, jint edge_factor)
{

    InitPool(&pool, 1000, 8);
    similarity_matrix = (double**) malloc(num_rows * sizeof(double*));
    for (int i = 0; i < num_rows; i++) {
      similarity_matrix[i] = (double*) malloc(num_rows * sizeof(double));
      for (int j = 0; j < num_rows; j++) {
        similarity_matrix[i][j] = -1.0; 
      }
    }


    num_nodes = num_rows;
    live_nodes = num_nodes;

    double threshold, antiseuil;
    mode_similitude = modeSimilitude;

    // un tableau dont de taille nombre de paire dans un ensemble avec num_rows element = 2 parmis num_rows
    double * similarities = (double*) malloc(num_rows * (num_rows - 1) / 2 * sizeof(double));
    double means_similitude = calculate_mean_similitude_parallel(modeSimilitude, similarities);

    calculate_threshold(modeSimilitude, 10*num_nodes, &threshold, &antiseuil, similarities);

    jclass res_class = (*env)->FindClass(env, "Lgraph/Metadata;");
    jmethodID constructor = (*env)->GetMethodID(env, res_class, "<init>", "(IDDD)V");
    jobject res = (*env)->NewObject(env, res_class, constructor, num_nodes, threshold, antiseuil, means_similitude);

    free(similarities);
    similarities = NULL;

    return res;
}


JNIEXPORT jobject JNICALL Java_graph_Graph_initializeGraph
  (JNIEnv *env, jobject obj, jint md, jdouble thresh, jdouble anti_thresh)
{

    calculate_similitude_and_edges(mode_similitude, thresh, anti_thresh);

    for (int i = 0; i < num_rows; i++) {
      free(similarity_matrix[i]);
    }
    free(similarity_matrix);
    similarity_matrix = NULL;

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
      init_S(num_nodes);
      num_communities = leiden_method_CPM();
      // Demander le chemin du fichier à l'utilisateur
      lireColonneCSV(S, &nbValeurs);
      modeA = 1;
      compute_ratio_S(S);
      free_S();
    }

    initialize_community_colors();

    for (int i = 0; i < num_nodes; i++) {
        random_point_in_center(i);
    }
    n_clusters = (int)sqrt(num_nodes);
    init_clusters(n_clusters);
    initialize_centers();
    assign_cluster_colors();
    calculate_node_degrees();

    jclass res_class = (*env)->FindClass(env, "Lgraph/Metadata;");
    jmethodID constructor = (*env)->GetMethodID(env, res_class, "<init>", "(IDDIII)V");
    jobject res = (*env)->NewObject(env, res_class, constructor, num_nodes, thresh, anti_thresh, num_edges, num_antiedges, n_clusters);

    thresholdS = (Lx/4000)*(Ly/4000);
    thresholdA = (Lx/4000)*(Ly/4000);
    epsilon = (Lx/800)*(Ly/800);
    seuilrep = (Lx/1000)*(Lx/1000);

    friction = 0.1;
    Max_movementOld = 0.;
    iteration = 0;
    espacement = 1;
    pause_updates = 0;

    return res;   

}

JNIEXPORT jobject JNICALL Java_graph_Graph_initializeDot
  (JNIEnv *env, jobject obj, jstring filepath, jint md)
{

  jboolean b = JNI_FALSE;
  const char* str = (*env)->GetStringUTFChars(env, filepath, &b);

  parse_dot_file(str);

  InitPool(&pool, 1000, 8);

  modeA = 0;
  if (md == 0) {
      num_communities = louvain_method();
  } else if (md == 1) {
      num_communities = louvain_methodC();
  } else if (md == 2) {
      num_communities = leiden_method();
  } else if (md == 3) {
      num_communities = leiden_method_CPM();
  } else if (md == 4) {
      init_S(num_nodes);
      num_communities = leiden_method_CPM();
      // Demander le chemin du fichier à l'utilisateur
      lireColonneCSV(S, &nbValeurs);
      modeA = 1;
      compute_ratio_S(S);
      free_S();
  } else {
      printf("Option invalide\n");
  }

  initialize_community_colors();
  for (int i = 0; i < num_nodes; i++) {
    random_point_in_center(i);
  }

  n_clusters = (int) sqrt(num_nodes);
  init_clusters(n_clusters);
  initialize_centers();
  assign_cluster_colors();
  calculate_node_degrees();

  jclass res_class = (*env)->FindClass(env, "graph/Metadata");
  jmethodID constructor = (*env)->GetMethodID(env, res_class, "<init>", "(IDDD)V");
  jobject res = (*env)->NewObject(env, res_class, constructor, num_nodes, 0., 0., 0.);

  (*env)->ReleaseStringUTFChars(env, filepath, str); 

  live_nodes = num_nodes;
  thresholdS = (Lx/4000)*(Ly/4000);
  thresholdA = (Lx/4000)*(Ly/4000);
  epsilon = (Lx/800)*(Ly/800);
  seuilrep = (Lx/1000)*(Lx/1000);

  friction = 0.1;
  Max_movementOld = 0.;
  iteration = 0;
  espacement = 1;
  pause_updates = 0;

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
        adjacency_list[i].head = NULL;
    }
    free_clusters();

    if ( similarity_matrix != NULL ) {
      for (int i = 0; i < num_rows; i++) {
        free(similarity_matrix[i]);
      }
      free(similarity_matrix);
    }
    freeNodeNames();
    FreePool(&pool);

    num_nodes = 0;
    live_nodes = 0;
    num_edges = 0;
    num_antiedges = 0;
}

JNIEXPORT void JNICALL Java_graph_Graph_freeProgramMemory
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
        adjacency_list[i].head = NULL;
    }
    free_clusters();
    if ( similarity_matrix != NULL ) {
      for (int i = 0; i < num_rows; i++) {
        free(similarity_matrix[i]);
      }
      free(similarity_matrix);
    }

    freeNodeNames();
    FreePool(&pool);

    if ( data != NULL ) {
      for (int i = 0; i < num_rows; ++i) {
        free(data[i]);
      }
      free(data);
    }

    num_nodes = 0;
    live_nodes = 0;
    num_edges = 0;
    num_antiedges = 0;
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

JNIEXPORT void JNICALL Java_graph_Graph_deleteNode
  (JNIEnv * env, jobject obj, jint index)
{

  if ( vertices[index].deleted == 0 ) {
    vertices[index].deleted = 1;
    modified_graph = 1;
    --live_nodes;
  }

}

JNIEXPORT void JNICALL Java_graph_Graph_restoreNode
(JNIEnv * env, jobject obj, jint index)
{

  if ( vertices[index].deleted == 1 ) {
    vertices[index].deleted = 0;
    modified_graph = 1;
    ++live_nodes;
  }

}

JNIEXPORT void JNICALL Java_graph_Graph_setKmeansMode
  (JNIEnv * env, jobject obj, jboolean b)
{
  kmeans_mode = b;
}

JNIEXPORT jobject JNICALL Java_graph_Graph_getHistogram
  (JNIEnv * env, jobject obj)
{
  jintArray result = (*env)->NewIntArray(env, NUM_BINS);

  (*env)->SetIntArrayRegion(env, result, 0, NUM_BINS,(const jint *) global_histogram);

  return result;
}

JNIEXPORT void JNICALL Java_graph_Graph_setInitialNodeSize
  (JNIEnv * env, jobject obj, jdouble size)
{
  initial_node_size = size;
}

JNIEXPORT void JNICALL Java_graph_Graph_setDegreeScaleFactor
  (JNIEnv * env, jobject obj, jdouble factor)
{
  degree_scale_factor = factor;
}
