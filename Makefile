CC = gcc-14
FLAGS = -Wall -g -lGL -lGLU -lglut -lm

DEBUG_DIR  = ./src/debug
DEBUG_DEPS = $(DEBUG_DIR)/%.h
DEBUG_SRC  = $(DEBUG_DIR)/%.c

DEBUG_OBJ  = debug_time.o 
DEBUG = $(patsubst %,$(DEBUG_DIR)/%,$(DEBUG_OBJ))
DEBUG_OUT = debug_forceatlas

CONCURRENT_DIR  = ./src/concurrent
CONCURRENT_DEPS = $(CONCURRENT_DIR)/%.h
CONCURRENT_SRC  = $(CONCURRENT_DIR)/%.c
CONCURRENT_OBJ  = Pool.o
CONCURRENT = $(patsubst %,$(CONCURRENT_DIR)/%,$(CONCURRENT_OBJ))

FORCE_ATLAS = ./src/forceatlasV4_CSV.c
OUT_NAME = forceatlas
DIR_SAMPLES = samples

$(DEBUG_DIR)/%.o: $(DEBUG_SRC) $(DEBUG_DEPS)
			$(CC) -c -o $@ $< $(FLAGS)

$(CONCURRENT_DIR)/%.o: $(CONCURRENT_SRC) $(CONCURRENT_DEPS)
			$(CC) -c -o $@ $< $(FLAGS)

all: $(CONCURRENT)
	$(CC) $(FORCE_ATLAS) -c
	$(CC) forceatlasV4_CSV.o $(CONCURRENT) -o $(OUT_NAME) $(FLAGS)

debug: $(DEBUG) $(CONCURRENT)
	$(CC) $(FORCE_ATLAS) -D_DEBUG_ -c
	$(CC) forceatlasV4_CSV.o $(DEBUG) $(CONCURRENT) -o $(DEBUG_OUT) $(FLAGS) -fsanitize=address

sample1:all
	./$(OUT_NAME) $(DIR_SAMPLES)/iris.csv

sample2:all
	./$(OUT_NAME) $(DIR_SAMPLES)/predicancerNUadd9239.csv

clean:
	- rm -rf $(OUT_NAME)
	- rm -rf $(DEBUG_OUT)
	- rm -rf *.o
	- rm -rf $(DEBUG_DIR)/*.o
	- rm -rf *.csv
	- rm -rf $(CONCURRENT_DIR)/*.o
	- rm -rf *.gexf
	- rm -rf *.txt
	- rm -rf *.png