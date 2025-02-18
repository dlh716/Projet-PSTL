FLAGS = -Wall -g -lGL -lGLU -lglut -lm
FORCE_ATLAS = ./src/forceatlasV4_CSV.c
OUT_NAME = forceatlas
DIR_SAMPLES = samples

all:
	gcc -o $(OUT_NAME) $(FORCE_ATLAS) $(FLAGS)

sample1:all
	./$(OUT_NAME) $(DIR_SAMPLES)/iris.csv

sample2:all
	./$(OUT_NAME) $(DIR_SAMPLES)/predicancerNUadd9239.csv

clean:
	- rm -rf $(OUT_NAME)
	- rm -rf *.gexf
	- rm -rf *.txt
	- rm -rf *.png