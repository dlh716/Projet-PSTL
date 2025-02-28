package com.mongraphe.graphcore;

import java.io.*;
import java.util.*;

import com.mongraphe.model.NodeProperties;


/**
 * Classe pour charger un graphe depuis un fichier avec un séparateur configurable.
 */
public class GraphLoader {

    /**
     * Charge un graphe à partir d'un fichier avec un séparateur configurable.
     * @param filePath Chemin du fichier à lire.
     * @param separator Caractère de séparation (ex: ',', ';', '\t').
     * @return Un objet Graph rempli avec les données du fichier.
     * @throws IOException Si une erreur survient en lisant le fichier.
     */
    public static Graph loadFromFile(String filePath, String separator) throws IOException {
        Graph graph = new GraphImpl();
        Map<String, Node> nodeMap = new HashMap<>();
        
        try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
            String line;
            boolean readingNodes = true; // Distingue les nœuds des arêtes
            List<String> nodeAttributes = null; // Liste pour stocker les attributs des nœuds

            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#")) continue; // Ignorer les lignes vides/commentaires

                String[] parts = line.split(separator);
                
                if (readingNodes) {
                    // Détection de la séparation entre nœuds et arêtes
                    if (parts[0].equalsIgnoreCase("source")) {
                        readingNodes = false;
                        continue;
                    }

                    // Si on est dans la section des nœuds, la première ligne contient les noms des attributs
                    if (nodeAttributes == null) {
                        nodeAttributes = Arrays.asList(parts); // On enregistre les noms des attributs
                        continue;
                    }

                    // Lecture des nœuds : id suivi des attributs dynamiques
                    String id = parts[0];
                    Node node = new Node(id);
                    NodeProperties properties = node.getProperties();

                    // Ajout des attributs dynamiques basés sur la première ligne
                    for (int i = 1; i < parts.length; i++) {
                        String attributeName = nodeAttributes.get(i);
                        properties.setAttribute(attributeName, parts[i]);
                    }

                    // On suppose que x et y se trouvent toujours à la fin des attributs (ou peuvent être dans n'importe quelle position)
                    if (parts.length > nodeAttributes.size()) {
                        properties.setX(Double.parseDouble(parts[parts.length - 2]));  // Position X
                        properties.setY(Double.parseDouble(parts[parts.length - 1]));  // Position Y
                    }

                    graph.addNode(node);
                    nodeMap.put(id, node);

                } else {
                    // Lecture des arêtes : source, target, weight, type
                    Node source = nodeMap.get(parts[0]);
                    Node target = nodeMap.get(parts[1]);
                    
                    if (source != null && target != null) {
                        Edge edge = graph.addEdge(source, target);
                        edge.setAttribute("weight", Double.parseDouble(parts[2]));
                        edge.setAttribute("type", parts[3]); // directed / undirected
                    }
                }
            }
        }
        return graph;
    }
}
