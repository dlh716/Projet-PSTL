#ifndef FORCEATLAS_DATA_H
#define FORCEATLAS_DATA_H

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <ctype.h>

short str_is_number(char* line);

void load_csv_data(const char *filename);
void lireColonneCSV(int *, int*);
void compute_ratio_S(int *);

void parse_dot_file(const char *filename);
void freeNodeNames();

#endif