#include "data.h"
#include "../global.h"

int nbValeurs;
double **data = NULL;
int num_rows = 0, num_columns = 0;
char delimiter[1] = "\0";
int S[MAX_NODES]={0};
char **node_names = NULL;

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

/**
 * We use this to store the name and the index of the node
 * in the vertices array
 */
struct HashPair {
    int index;
    char* label;
};

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

void find_parameters
    (char** line, size_t* size, double* weight, char** node_name) 
{

    while (*size > 0) {
        char* next = next_str(line, size);

        if ( weight != NULL && *size > 0 && strcmp(next, "weight") == 0 ) {
            char* next = next_str(line, size);
            if ( next != NULL ) {
                sscanf(next, "%lf", weight);
                free(next);
            }
        } else if ( node_name != NULL && *size > 0 && strcmp(next, "label") == 0 ) {
            char* next = next_str(line, size);
            if ( next != NULL ) {
                *node_name = next;
            }
        }
    }
    
}

int hash_string(char* label) {

    int cpt = 0;
    while ( *label != '\0' ) {
        cpt += (int) *label;
        ++label;
    }

    return cpt;
}

int map_put(struct HashPair** map, size_t* capacity, char* key, int size_key) {
    
    if ( *capacity <= num_nodes ) {
        size_t new_capacity = 2 * *capacity;
        struct HashPair* new_map = (struct HashPair*) malloc(sizeof(struct HashPair) * new_capacity);
        for (int i = 0; i < new_capacity; ++i) {
            new_map[i].label = NULL;
            new_map[i].index = -1;            
        }
        for (int i = 0; i < *capacity; ++i) {
            int new_index = hash_string((*map)[i].label) % new_capacity;
            while( new_map[new_index].index != -1 ) {
                new_index = (new_index + 1) % *capacity;
            }
            new_map[new_index].label = (*map)[i].label;
            new_map[new_index].index = (*map)[i].index;
        }
        node_names = (char**) realloc(node_names, sizeof(char*) * new_capacity);
        free(*map);
        *map = new_map;
        *capacity = new_capacity;
    }

    int ind = hash_string(key) % *capacity;

    while( (*map)[ind].index != -1 && strcmp(key, (*map)[ind].label) != 0 ) {
        ind = (ind + 1) % *capacity;
    }

    if ( (*map)[ind].index == -1 ) {
        (*map)[ind].index = num_nodes++;
        (*map)[ind].label = (char*) malloc(sizeof(char) * (size_key + 1));
        strncpy((*map)[ind].label, key, size_key * sizeof(char));
        (*map)[ind].label[size_key] = '\0';
        printf("LABEL: %d %s\n", (*map)[ind].index, (*map)[ind].label);
    }
    return (*map)[ind].index;
}

int belongs(char* str, const char** keywords, size_t size) {
    for (int i = 0; i < size; ++i) {
        if ( strcmp(str, keywords[i]) == 0 ) {
            return 1;
        }
    }
    return 0;
}

int isKeyword(char* str) {
    const char* keywords[] = {
        "digraph", "strict", "graph", "subgraph",
        "node", "edge", "nodesep", "ranksep", "margin" 
    };
    size_t nb_keywords = 9;
    return belongs(str, keywords, nb_keywords);
}

int isCompass(char* str) {
    const char* compass[] = {
        "n", "ne", "nw", "s", "se", "sw", "e", "w", "c", "_"
    };
    size_t nb_compass = 10;
    return belongs(str, compass, nb_compass);
}

int isDelimiter(char c) {
    const char delimiters[] = {
        ' ', '[', ']', '{', '}', '=', ';', ',', ':', '"', '-', '>', '\n', '\r'
    };
    size_t nb_delimiter = 14;
    for (int i = 0; i < nb_delimiter; ++i) {
        if ( delimiters[i] == c ) {
            return 1;
        }
    }
    return 0;
}

int isHtmlString(const char *s, size_t size) {
    return size >= 2 && s[0] == '<' && s[size-1] == '>';
}

int isQuoteString(const char* s, size_t size) {

    if ( size < 2 && s[0] != '"' && s[size-1] != '"' ) {
        return 0;
    }

    for (int i = 0; i < size - 1; ++i) {
        if ( s[i] == '\\' && s[i+1] == '"' ) {
            ++i;
        } else if ( s[i] == '"' ) {
            return 0;
        }
    }
    return 1;
}

int isAlpha(const char* s, size_t size) {
    if ( s[0] == '_' && isdigit(s[0]) )
        return 0;

    for (int i = 0; i < size; ++i) {
        if ( ! isalnum(s[i]) && (200 > s[i] || s[i] > 255) && s[i] != '_' )
            return 0;
    }
    return 1;
}

/**
 * checks if s respects the Identifier format of dot files
 * s should be null terminated
 */
int isId(char *s, size_t size) {
    return 
    isHtmlString(s, size) 
    || str_is_number(s)
    || isQuoteString(s, size)
    || isAlpha(s, size);
}

int isParameter(char* s) {
    const char* parameters[] = {
        "weight", "label"
    };
    if ( strcmp(s, parameters[0]) == 0 )
        return 1;
    else if ( strcmp(s, parameters[1]) == 0 ) 
        return 2;
    return 0;
}

int next_token(FILE* file, char * buffer, size_t* size, int inQuotes) {

    int c;
    *size = 0;
    while ( (c = fgetc(file)) != EOF  ) {
        if ( (inQuotes && (c != '"' || (*size > 0 && buffer[*size-1] == '\\')) ) 
        || ! isDelimiter(c) ) {
            buffer[*size] = c;
            ++*size;
       } else {
            break;
        }
    }

    buffer[*size] = '\0';

    return c;
}

struct vect {
    int * content;
    int capacity, size;
};
typedef struct vect* Vect;



void add_elem_list(
    Vect nodes,
    int elem)
{
    if ( nodes->size >= nodes->capacity ) {
        nodes->content = (int*) realloc(nodes->content, nodes->capacity * sizeof(int) * 2);
        nodes->capacity *= 2;
    } 
    (nodes->content)[nodes->size] = elem;
    ++nodes->size;
}

int add_new_edge(
    Vect nodes,
    Vect tmp) 
{
    if ( tmp ) {
        for (int i = 0; i < tmp->size; ++i) {
            if ( num_edges >= MAX_EDGES ) return -1;
            edges[num_edges].node1 = tmp->content[i];
            edges[num_edges].node2 = nodes->content[nodes->size-1];
            ++num_edges;
        }
    } else if ( num_edges < MAX_EDGES ){
        edges[num_edges].node1 = (nodes->content)[nodes->size-2];
        edges[num_edges].node2 = (nodes->content)[nodes->size-1];
        ++num_edges;
    } else {
        return -1;
    }

    return 0;
}

struct vect newVect(int capacity) {
    struct vect res;
    res.capacity = capacity;
    res.content = (int*) malloc(sizeof(int) * capacity);
    res.size = 0;
    return res;
} 

void freeVect(Vect v) {

    if ( v != NULL ) {
        if ( v->content != NULL ) {
            free(v->content);
        }
        free(v);
    }

}

void parse_attr(FILE *file, char* buffer, double* weight, char** label) {

    size_t size = 0;
    char c = next_token(file, buffer, &size, 0);
    while ( c != EOF && c != ']') {
        int id = isParameter(buffer);
        printf("ATTR: %s\n", buffer);
        if ( id ) {
            c = next_token(file, buffer, &size, 0);
            while ( size == 0 && c != '"') {
                c = next_token(file, buffer, &size, 0);
            }

            if ( id == 1 ) {
                *weight = atof(buffer);
            } else {
                if ( c == '"' ) {
                    c = next_token(file, buffer, &size, 1);
                }

                *label = (char*) malloc(sizeof(char) * (size + 1));
                (*label)[size] = '\0';
                for(int i = 0; i < size; ++i) {
                    (*label)[i] = buffer[i];
                }  
            }

        }
        c = next_token(file, buffer, &size, 0);
    }

}

Vect parse_stmt_list(
    FILE *file, 
    char* buffer, 
    struct HashPair** map, 
    size_t* capacity) {

    size_t size = 0;
    int flag = 1;
    int side_edge = 0;
    int ignore = 0;

    struct vect node_list1 = newVect(4);
    Vect node_list_tmp1 = NULL;
    Vect node_list_tmp2 = NULL;

    int last_edge = num_edges;

    int edge_overflow = 0;

    while ( flag ) {
        int c = next_token(file, buffer, &size, 0);
        if ( size > 0 && ! isKeyword(buffer) && isId(buffer, size) && ! isCompass(buffer) ) {

            if ( ! ignore ) {
                int index = map_put(map, capacity, buffer, size);
                
                if ( ! side_edge ) {
                    for(int i = 0; node_list_tmp1 && i < node_list_tmp1->size; ++i) {
                        add_elem_list(&node_list1, node_list_tmp1->content[i]);
                    }
                    freeVect(node_list_tmp1);
                    node_list_tmp1 = NULL;
                }

                add_elem_list(&node_list1, index);
                last_edge = num_edges;

                if ( side_edge ) {
                    edge_overflow |= add_new_edge(&node_list1, node_list_tmp1);
                    freeVect(node_list_tmp1);
                    node_list_tmp1 = NULL;
                }
                side_edge = 0;
            } else ignore = 0;
        }

        switch (c) {
        case '{':
            node_list_tmp2 = parse_stmt_list(file, buffer, map, capacity);
            if ( side_edge ) {
                last_edge = num_edges; 
                if ( node_list_tmp1 ) {
                    for (int i = 0; ! edge_overflow && i < node_list_tmp1->size; ++i) {
                        for (int j = 0; j < node_list_tmp2->size; ++j) {
                            if ( num_edges >= MAX_EDGES ) {
                                edge_overflow = 1;
                                break;
                            }
                            edges[num_edges].node1 = node_list_tmp1->content[i];
                            edges[num_edges].node2 = node_list_tmp2->content[j];
                            ++num_edges;
                        }
                    }

                } else {
                    for (int i = 0; ! edge_overflow && i < node_list_tmp2->size; ++i) {
                        if ( num_edges >= MAX_EDGES ) {
                            edge_overflow = 1;
                        } else {
                            int last_list1 = node_list1.size-1;
                            edges[num_edges].node1 = node_list1.content[last_list1];
                            edges[num_edges].node2 = node_list_tmp2->content[i];
                            ++num_edges;
                        }
                    }
                }
                side_edge = 0;
            }
            for(int i = 0; node_list_tmp1 && i < node_list_tmp1->size; ++i) {
                add_elem_list(&node_list1, node_list_tmp1->content[i]);
            }
            freeVect(node_list_tmp1);
            node_list_tmp1 = node_list_tmp2;
            node_list_tmp2 = NULL; 
            break;
        case '-':
            c = next_token(file, buffer, &size, 0);
            if ( c == '>' || c == '-' ) {
                side_edge = 1;
            }
            break;
        case '"':
            next_token(file, buffer, &size, 1);
            break;
        case '[':
            double w = 1.0;
            char * label = NULL;
            parse_attr(file, buffer, &w, &label);
            
            for (int i = last_edge; i < num_edges; ++i) {
                edges[i].weight = w;
            }

            if ( label != NULL && last_edge == num_edges && node_list1.size > 0 ) {
                int last_list1 = node_list1.size-1;
                node_names[node_list1.content[last_list1]] = label;
            }

            break;
        case '=':
            ignore = 1;
            break;
        case '}':
        case EOF:
            flag = 0;
        default:
            break;
        }
    }

    for(int i = 0; node_list_tmp1 && i < node_list_tmp1->size; ++i) {
        add_elem_list(&node_list1, node_list_tmp1->content[i]);
    }
    freeVect(node_list_tmp1);

    Vect res = (Vect) malloc(sizeof(struct vect));
    res->content = node_list1.content;
    res->capacity = node_list1.capacity;
    res->size = node_list1.size;

    return res;
}

#define MAXBUF 1024

void parse_dot_file(const char* filename) {
    FILE *file = fopen(filename, "r");
    
    if (file == NULL) {
        perror("Error opening file");
    }

    char buf[MAXBUF];
    size_t size = 0;

    int c = next_token(file, buf, &size, 0);
    while ( c != EOF && c != '{' ) {
        c = next_token(file, buf, &size, 0);
    }
    struct HashPair* map = (struct HashPair*) malloc(sizeof(struct HashPair) * MAX_NODES);
    size_t capacity = MAX_NODES;
    node_names = (char**) malloc(sizeof(char*) * capacity);
    for (int i = 0; i < capacity; ++i) {
        map[i].index = -1;
        map[i].label = NULL;
        node_names[i] = NULL;
    }

    switch(c) {
        case '{':
            Vect res = parse_stmt_list(file, buf, &map, &capacity);
            freeVect(res);
            break;
        default:
            break;    
    }
    
    free(map);
    fclose(file);
    printf("END\n");
}

void freeNodeNames() {
    if ( node_names != NULL ) {

        for (int i = 0; i < num_rows; ++i) {
            if ( node_names[i] != NULL ) {
                free(node_names[i]);
            }
        }
        free(node_names);
    }
}