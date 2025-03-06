CC = gcc-14
FLAGS = -Wall -g -lGL -lGLU -lglut -fPIC -lm -lc -shared
JNI_FLAGS = -I/usr/lib/jvm/java-21-openjdk-amd64/include -I/usr/lib/jvm/java-21-openjdk-amd64/include/linux -fPIC

SRC_DIR = src
OUT_DIR = out

JAVAFX_DIR = lib/javafx-sdk-23.0.2/lib
JAVAFX_MODULES = javafx.controls,javafx.fxml

GRAPH_JAVA = $(SRC_DIR)/*/*.java
GRAPH_JAVA_MAIN = $(SRC_DIR)/graph/Graph.java
GRAPH_JAVA_OUT = -cp $(OUT_DIR) graph.Graph

DEBUG_DIR  = debug
DEBUG_DEPS = $(DEBUG_DIR)/%.h

CONCURRENT_DIR  = $(SRC_DIR)/concurrent
CONCURRENT_DEPS = $(CONCURRENT_DIR)/%.h
CONCURRENT_SRC  = $(CONCURRENT_DIR)/%.c
CONCURRENT_OBJ  = Pool.o
CONCURRENT = $(patsubst %,$(CONCURRENT_DIR)/%,$(CONCURRENT_OBJ))

FORCE_ATLAS = $(SRC_DIR)/c/forceatlasV4_CSV.c
FORCE_ATLAS_OUT = $(OUT_DIR)/forceatlas.o
LIBNATIVE_OUT = $(OUT_DIR)/libnative.so

OUT_NAME = forceatlas

DIR_SAMPLES = samples
DIR_SAMPLE = $(DIR_SAMPLES)/iris.csv

$(DEBUG_DIR)/%.o: $(DEBUG_SRC) $(DEBUG_DEPS)
	$(CC) -c -o $@ $< $(FLAGS)

$(CONCURRENT_DIR)/%.o: $(CONCURRENT_SRC) $(CONCURRENT_DEPS)
	$(CC) -c -o $@ $< $(FLAGS)

all: $(CONCURRENT)
	javac --module-path $(JAVAFX_DIR) --add-modules $(JAVAFX_MODULES) -d $(OUT_DIR) $(GRAPH_JAVA)
	javac --module-path $(JAVAFX_DIR) --add-modules $(JAVAFX_MODULES) -h $(OUT_DIR) -d $(OUT_DIR) $(GRAPH_JAVA)

	$(CC) $(JNI_FLAGS) -c $(FORCE_ATLAS) -o $(FORCE_ATLAS_OUT)
	$(CC) $(FORCE_ATLAS_OUT) $(CONCURRENT) -o $(LIBNATIVE_OUT) $(FLAGS)

	java -Djava.library.path=. --module-path $(JAVAFX_DIR) --add-modules $(JAVAFX_MODULES) $(GRAPH_JAVA_OUT) $(DIR_SAMPLE)

clean:
	- rm -rf $(OUT_DIR)/*.o
	- rm -rf $(OUT_DIR)/*.h
	- rm -rf $(OUT_DIR)/*/*.class
	- rm -rf $(SRC_DIR)/*/*.o
	- rm -rf $(shell find out -mindepth 1 -type d)