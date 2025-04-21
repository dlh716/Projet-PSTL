package graph;

/**
 * Définit les constantes utilisées pour les différentes opérations du graphe
 */
public interface GraphData {

    enum SimilitudeMode { CORRELATION, DISTANCE_COSINE, DISTANCE_EUCLIDIENNE, NORME_L1, NORME_LINF, KL_DIVERGENCE }

    enum NodeCommunity { LOUVAIN, LOUVAIN_PAR_COMPOSANTE, LEIDEN, LEIDEN_CPM, COULEURS_SPECIALES }

    /** RUN : Exécution du graphe (en mouvement)
     *  SELECTION : Permet de sélectionner et déplacer des sommets
     *  MOVE : Permet de se déplacer dans le graphe
     *  DELETE : Permet de supprimer des sommets */
    enum GraphMode { RUN, SELECTION, MOVE, DELETE }

}
