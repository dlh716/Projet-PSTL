#include "data.h"
#include "../global.h"

int nbValeurs;
double **data = NULL;
int num_rows = 0, num_columns = 0;
char delimiter[1] = "\0";
int S[MAX_NODES]={0};

short str_is_number(char* line)
{
    if ( *line == '-' ) {
        ++line;
    } else if ( *line == '\0' ) {
        return 0;
    }

    short pt = 0;
    while ( (*line >= 48 && *line <= 57) || (*line == 46 && ! pt) || *line == 101 ) {
        if (*line == 46)
            pt = 1;
        else if (*line == 101)
            return str_is_number(++line);
        ++line;
    }

    return *line == '\0';
}

// chargement Data
void load_csv_data(const char *filename) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        printf("Could not open file %s\n", filename);
        exit(1);
    }

    char *line = NULL;
    size_t len = 0;

    // Première passe : Compter les lignes et les colonnes
    while (getline(&line, &len, file) != -1) {
        // Supprimer les retours à la ligne potentiels
        line[strcspn(line, "\r\n")] = '\0';
        num_rows++;

        // Compter les colonnes uniquement à la première ligne
        if (num_columns == 0) {
            int count = 0;  // Commence à 1 car il y a toujours une colonne avant la première virgule
            for (int i = 0; line[i] != '\0'; i++) {
                if (delimiter[0] == '\0' || line[i] == ',' || line[i] == ';' || line[i] == '|' || line[i] == ' ' || line[i] == '\t') {
                    delimiter[0] = line[i];
                    count++;
                } else if (line[i] == delimiter[0]) {
                    count++;
                }
            }

            num_columns = count;
        }
    }

    // Allouer de la mémoire pour les données
    data = (double **)malloc((num_rows-1) * sizeof(double *));
    for (int i = 0; i < num_rows-1; i++) {
        data[i] = (double *)malloc(num_columns * sizeof(double));
    }

    // Deuxième passe : Lire les données
    rewind(file);
    int row = 0;
    while (getline(&line, &len, file) != -1) {
        line[strcspn(line, "\r\n")] = '\0';  // Supprimer les retours à la ligne potentiels

        int col = 0;
        char *start = line;
        char *end = NULL;

        // Parcourir chaque valeur séparée par des virgules
        while ((end = strchr(start, delimiter[0])) != NULL || *start != '\0') {
            if (end) {
                *end = '\0';  // Terminer la chaîne courante à la virgule
            }
            int is_number = str_is_number(start);
            // Convertir la valeur en double
            if ( row > 0 && ! is_number ){
                printf("Warning %s: Missing value on row %d, col %d\n", start, row, col);
            }
            if ( row > 0 && is_number ) {
                data[row-1][col] = atof(start);
            } else if ( row > 0 ) {
                data[row-1][col] = 0.;
            }
            col++;

            // Si end est NULL, on est à la dernière valeur
            if (!end) break;

            start = end + 1;  // Passer à la prochaine valeur
        }

        // Si le nombre de colonnes lues est différent de num_columns, avertir
        if (col != num_columns) {
            //printf("Warning: Row %d has %d columns (expected %d).\n", row, col, num_columns);
        }

        row++;
    }
    
    fclose(file);
    free(line);  // Libérer la mémoire allouée par getline

    printf("Loaded CSV with %d rows and %d columns.\n", num_rows, num_columns);
    
        // Afficher les trois premières lignes
    printf("Les 3 premières lignes des données :\n");
    for (int i = 0; i < 5 && i < num_rows; i++) {
        for (int j = 0; j < num_columns; j++) {
            printf("%f ", data[i][j]);
        }
        printf("\n");
    }

    // On ne garde pas la première ligne dans les données
    num_rows = num_rows - 1;
}


// Fonction pour lire les valeurs de la première colonne d'un fichier CSV S[MAX_NODES]
void lireColonneCSV(int *S,int *nbValeurs) {
    char chemin[256];
    printf("Veuillez entrer le chemin du fichier CSV: ");
    // Use scanf with a width specifier to avoid buffer overflow
    scanf("%255s", chemin);  // Limiting the input to 255 characters
    FILE *fichier = fopen(chemin, "r");
    if (fichier == NULL) {
        perror("Erreur lors de l'ouverture du fichier");
        return;
    }

    char ligne[MAX_LINE_LENGTH];
    *nbValeurs = 0; // Initialisation du nombre de valeurs

    // Lecture de chaque ligne du fichier
    while (fgets(ligne, sizeof(ligne), fichier) != NULL) {
        // Séparer la première colonne en utilisant le point-virgule comme délimiteur
        char *token = strtok(ligne, delimiter);
        if ( token != NULL && str_is_number(token) ) {
            // Vérifier si le tableau S n'est pas plein
            if (*nbValeurs < MAX_NODES) {
                // Convertir la chaîne en entier et stocker dans le tableau S
                if (str_is_number(token)) {
                    S[*nbValeurs] = atoi(token);
                } else {
                    S[*nbValeurs] = 0;
                }
                //printf("S[%d]=%d ",*nbValeurs,S[*nbValeurs]);
                (*nbValeurs)++;
            } else {
                printf("Nombre maximum de valeurs atteint.\n");
                break;
            }
        } else {
            
            printf("Missing values in csv file");
        }
    }
    // Afficher les trois premières lignes
    printf("Les 5 premières lignes des données :\n");
    for (int i = 0; i < 5 && i < num_rows; i++) {
            printf("%d \n", S[i]);
    }
    fclose(fichier);
}

void parse_dot_file(const char *filename) {
    printf("fichier %s bientôt ouvert",filename);
    fflush(stdout);
    FILE *file = fopen(filename, "r");
    if (file == NULL) {
        perror("Error opening file");
        return;
    }

    char line[10000];
    //int current_node = 0;
    printf("fichier %s ouvert",filename);
    fflush(stdout);
    while (fgets(line, sizeof(line), file)) {
        // Remove leading and trailing whitespace
        char *start = line;
        while (isspace(*start)) start++;
        char *end = start + strlen(start) - 1;
        while (end > start && isspace(*end)) end--;
        *(end + 1) = '\0';

        int node1, node2;
        double weight = 1.0; // Default weight if not provided

        // Check for directed edges (->)
        if (strstr(start, "->")) {
            // Extract nodes for directed edge
            if (sscanf(start, "%d -> %d", &node1, &node2) == 2) {
                // Update num_nodes
                if (node1 >= num_nodes) num_nodes = node1 + 1;
                if (node2 >= num_nodes) num_nodes = node2 + 1;

                // Add directed edge
                if (num_edges < MAX_EDGES) {
                    edges[num_edges].node1 = node1;
                    edges[num_edges].node2 = node2;
                    edges[num_edges].weight = weight;
                    num_edges++;
                } else {
                    fprintf(stderr, "Warning: Number of edges exceeds MAX_EDGES\n");
                }
            } else {
                fprintf(stderr, "Warning: Failed to parse directed edge line: %s\n", start);
            }
        }
        // Check for undirected edges (--)
        else if (strstr(start, "--")) {
            // Extract nodes for undirected edge
            if (sscanf(start, "%d -- %d", &node1, &node2) == 2) {
                // Update num_nodes
                if (node1 >= num_nodes) {num_nodes = node1 + 1;}
                if (node2 >= num_nodes) {num_nodes = node2 + 1;}

                // Add undirected edge (could be treated the same as directed in storage)
                if (num_edges < MAX_EDGES) {
                    edges[num_edges].node1 = node1;
                    edges[num_edges].node2 = node2;
                    edges[num_edges].weight = weight;
                    num_edges++;
                } else {
                    fprintf(stderr, "Warning: Number of edges exceeds MAX_EDGES\n");
                }
            } else {
                fprintf(stderr, "Warning: Failed to parse undirected edge line: %s\n", start);
            }
        }
    }

    fclose(file);
}