package graph;

/**
 * Définit les constantes utilisées pour les différentes opérations du graphe
 */
public interface GraphData {

    enum SimilitudeMode { CORRELATION, DISTANCE_COSINE, DISTANCE_EUCLIDIENNE, NORME_L1, NORME_LINF, KL_DIVERGENCE }

    enum NodeCommunity { LOUVAIN, LOUVAIN_PAR_COMPOSANTE, LEIDEN, LEIDEN_CPM, COULEURS_SPECIALES }

    enum RepulsionMode { REPULSION_BY_DEGREE, REPULSION_BY_EDGES, REPULSION_BY_COMMUNITIES }

    /** <p><code>RUN</code> : Exécution du graphe (en mouvement)</p>
     *  <p><code>SELECTION</code> : Permet de sélectionner et déplacer des sommets</p>
     *  <p><code>MOVE</code> : Permet de se déplacer dans le graphe</p>
     *  <p><code>DELETE</code> : Permet de supprimer des sommets</p> */
    enum GraphMode { RUN, SELECTION, MOVE, DELETE }

}
