#include "data.h"
#include "../global.h"

int nbValeurs;
double **data = NULL;
int num_rows = 0, num_columns = 0;
char delimiter[1] = "\0";
int S[MAX_NODES]={0};
char *node_names[MAX_NODES];

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

void split_str(char * line, size_t size, char* delimiters, int nb_delimiter) {
    
    for (int i = 0; i < size; ++i) {

        for (int j = 0; j < nb_delimiter; ++j) {
            if ( *line == delimiters[j] ) {
                *line = '\0';
            }
        }
        ++line;
    }

}

char* next_str(char** line, size_t* s) {

    while ( **line == '\0' ) { 
        ++*line; 
        --*s;
    }

    char * start = *line;
    int cpt = 0;
    while ( **line != '\0' && *s > 0) {
        ++cpt;
        ++*line;
        --*s;
    }

    char* res = NULL;
    if ( cpt != 0 ) {

        res = (char*) malloc(sizeof(char) * (cpt + 1));
        for (int i = 0; i < cpt; ++i) {
            res[i] = start[i];
        }
        res[cpt] = '\0';
    }

    return res;
}

char* find_parameter(char** line, const char* parameter, size_t* size) {

    while (*size > 0) {
        char* next = next_str(line, size);

        if ( *size > 0 && strcmp(next, parameter) == 0 ) {
            printf("parameter: %s %ld\n", next, *size);
            return next_str(line, size);
        }
    }
    
    return NULL;
}

int hash_string(char* label) {

    int cpt = 0;
    while ( *label != '\0' ) {
        cpt += (int) *label;
        ++label;
    }

    return cpt;
}

int put_node_names(int index, char *label) {

    index = index % MAX_NODES;
    int cpt = 0;
    while ( node_names[index] != NULL ) {
        if ( strcmp(node_names[index], label) == 0 ) {
            free(label);
            return index;
        }

        index = (index + 1) % MAX_NODES;

        if ( ++cpt == MAX_NODES ) {
            free(label);
            return -1;
        }
    }

    node_names[index] = label;
    ++num_nodes;

    return index;
}

void parse_dot_line(char * ptr, size_t size) {

    int node1 = -1, node2;
    double weight = 1.0;

    if ( strstr(ptr, "->") || strstr(ptr, "--") ) {
        char delimiters[5] = {' ', '[', ']', '=', ';'};

        split_str(ptr, size, delimiters, 5);
            
        // reading the first node label
        char* next = next_str(&ptr, &size);
        if ( next != NULL )
            node1 = put_node_names(hash_string(next), next);
        if ( node1 == -1 ) { 
            fprintf(stderr, "Warning: Maximum number of nodes reached\n");
            return ; 
        }
            
        // we can ignore it as it should be either "->" or "--"
        next = next_str(&ptr, &size);
        free(next); 


        next = next_str(&ptr, &size);
        if ( next != NULL )
            node2 = put_node_names(hash_string(next), next);
        if ( node2 == -1 ) { 
            fprintf(stderr, "Warning: Maximum number of nodes reached\n");
            return ; 
        }

        next = find_parameter(&ptr, "weight", &size);
        if ( next != NULL ) {
            sscanf(next, "%lf", &weight);
            free(next);
        }
        
        if ( num_edges < MAX_EDGES ) {
            edges[num_edges].node1 = node1;
            edges[num_edges].node2 = node2;
            edges[num_edges].weight = weight;

            ++num_edges;
        } else {
            fprintf(stderr, "Warning: Number of edges exceeds MAX_EDGES\n");
            return ;
        }
            
    }

}

void parse_dot_file(const char *filename) {
    FILE *file = fopen(filename, "r");
    printf("%s\n", filename);
    if (file == NULL) {
        perror("Error opening file");
        exit(1);
    }

    for (int i = 0; i < MAX_NODES; ++i){
        node_names[i] = NULL;
    }

    char* line = NULL;
    size_t size = 0;

    while ( getline(&line, &size, file) != -1 ) {
        parse_dot_line(line, size);
        free(line);
        line = NULL;
    }

    printf("FIN: %d", num_nodes);
    fflush(NULL);

    fclose(file);
}

void replace_edge_index(int index, int new_id) {

    for (int i = 0; i < num_edges; ++i) {
        if ( edges[i].node1 == index ) {
            edges[i].node1 = new_id;
        } 
        if ( edges[i].node2 == index ) {
            edges[i].node2 = new_id;
        }
    }

}

void compact_label() {

    int cpt = 0;
    for (int i = 0; i < MAX_NODES && cpt < num_nodes; ++i) {

        if ( node_names[i] == NULL ) {

            int flag = 1;
            for (int j = i + 1; flag && j < MAX_NODES; ++j) {
                if ( node_names[j] != NULL ) {
                    node_names[i] = node_names[j];
                    node_names[j] = NULL;
                    replace_edge_index(j, i);
                    flag = 0;
                }
            }

        }

        cpt += 1;

    }

}