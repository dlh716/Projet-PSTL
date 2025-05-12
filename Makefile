CC = gcc
FLAGS = -Wall -g -lGL -lGLU -lglut -fPIC -lm -lc -shared
JNI_FLAGS = -I/usr/lib/jvm/java-21-openjdk-amd64/include -I/usr/lib/jvm/java-21-openjdk-amd64/include/linux -fPIC

SRC_DIR = src
OUT_DIR = out

JAVAFX_DIR = lib/javafx-sdk-23.0.2/lib
JAVAFX_MODULES = javafx.controls,javafx.fxml,javafx.swing

# JavaFX exportation et ouverture de modules internes
JAVA_EXPORTS = \
	--add-exports javafx.graphics/com.sun.javafx.scene=ALL-UNNAMED \
	--add-exports javafx.graphics/com.sun.javafx.tk=ALL-UNNAMED \
	--add-exports javafx.graphics/com.sun.javafx.application=ALL-UNNAMED \
	--add-exports javafx.graphics/com.sun.glass.ui=ALL-UNNAMED \
	--add-exports javafx.graphics/com.sun.javafx.tk.quantum=ALL-UNNAMED \
	--add-opens javafx.graphics/javafx.stage=ALL-UNNAMED \
	--add-opens javafx.base/javafx.collections=ALL-UNNAMED \
	--add-opens javafx.graphics/com.sun.javafx.tk=ALL-UNNAMED \
	--add-opens javafx.graphics/com.sun.glass.ui=ALL-UNNAMED \
	--add-opens javafx.graphics/com.sun.javafx.application=ALL-UNNAMED

# Configuration de JOGL
JOGL_DIR = lib/jogl
JOGL_JARS = $(JOGL_DIR)/jogl-all.jar:$(JOGL_DIR)/gluegen-rt.jar
JOGL_NATIVE_LIBS_LINUX = $(JOGL_DIR)/jogl-all-natives-linux-amd64.jar:$(JOGL_DIR)/gluegen-rt-natives-linux-amd64.jar
JOGL_NATIVE_LIBS_WINDOWS = $(JOGL_DIR)/jogl-all-natives-windows-amd64.jar:$(JOGL_DIR)/gluegen-rt-natives-windows-amd64.jar
JOGL_NATIVE_LIBS_MACOS = $(JOGL_DIR)/jogl-all-natives-macosx-universal.jar:$(JOGL_DIR)/gluegen-rt-natives-macosx-universal.jar
CLASSPATH = $(JOGL_JARS):$(JOGL_NATIVE_LIBS_LINUX):$(OUT_DIR)

GRAPH_JAVA = $(SRC_DIR)/*/*.java
GRAPH_JAVA_MAIN = $(SRC_DIR)/graph/Graph.java
GRAPH_JAVA_OUT = -cp $(CLASSPATH) graph.Graph

GRAPH_C_DIR  = $(SRC_DIR)/c/c_graph
GRAPH_C_DEPS = $(GRAPH_C_DIR)/%.h
GRAPH_C_SRC  = $(GRAPH_C_DIR)/%.c
GRAPH_C_OBJ  =  graph.o cluster.o communities.o
GRAPH_C      = $(patsubst %, $(GRAPH_C_DIR)/%,$(GRAPH_C_OBJ))

DEBUG_DIR  = $(SRC_DIR)/c/debug
DEBUG_DEPS = $(DEBUG_DIR)/%.h
DEBUG_SRC = $(DEBUG_DIR)/%.c
DEBUG_OBJ = debug_time.o
DEBUG = $(patsubst %, $(DEBUG_DIR)/%,$(DEBUG_OBJ))

CONCURRENT_DIR  = $(SRC_DIR)/c/concurrent
CONCURRENT_DEPS = $(CONCURRENT_DIR)/%.h
CONCURRENT_SRC  = $(CONCURRENT_DIR)/%.c
CONCURRENT_OBJ  = Pool.o Barrier.o Tools.o
CONCURRENT = $(patsubst %,$(CONCURRENT_DIR)/%,$(CONCURRENT_OBJ))

PRETRAITEMENT_DIR = $(SRC_DIR)/c/pretraitement
PRETRAITEMENT_DEPS = $(PRETRAITEMENT_DIR)/%.h
PRETRAITEMENT_SRC = $(PRETRAITEMENT_DIR)/%.c
PRETRAITEMENT_OBJ = data.o similarity.o
PRETRAITEMENT = $(patsubst %,$(PRETRAITEMENT_DIR)/%,$(PRETRAITEMENT_OBJ))

FORCE_ATLAS = $(SRC_DIR)/c/forceatlasV4_CSV.c
FORCE_ATLAS_OUT = $(OUT_DIR)/forceatlas.o
LIBNATIVE_OUT = $(OUT_DIR)/libnative.so

OUT_NAME = forceatlas

DIR_SAMPLES = samples
DIR_SAMPLE = $(DIR_SAMPLES)/iris.csv

$(DEBUG_DIR)/%.o: $(DEBUG_SRC) $(DEBUG_DEPS)
	$(CC) -c -o $@ $< $(FLAGS) $(JNI_FLAGS)

$(CONCURRENT_DIR)/%.o: $(CONCURRENT_SRC) $(CONCURRENT_DEPS)
	$(CC) -c -o $@ $< $(FLAGS) $(JNI_FLAGS)

$(GRAPH_C_DIR)/%.o: $(GRAPH_C_SRC) $(GRAPH_C_DEPS)
	$(CC) -c -o $@ $< $(FLAGS) $(JNI_FLAGS)

$(PRETRAITEMENT_DIR)/%.o: $(PRETRAITEMENT_SRC) $(PRETRAITEMENT_DEPS)
	$(CC) -c -o $@ $< $(FLAGS) $(JNI_FLAGS)

all: $(CONCURRENT) $(GRAPH_C) $(PRETRAITEMENT)
	javac --module-path $(JAVAFX_DIR) --add-modules $(JAVAFX_MODULES) -cp $(CLASSPATH) -d $(OUT_DIR) $(GRAPH_JAVA)
	javac --module-path $(JAVAFX_DIR) --add-modules $(JAVAFX_MODULES) -cp $(CLASSPATH) -h $(OUT_DIR) -d $(OUT_DIR) $(GRAPH_JAVA)

	$(CC) $(JNI_FLAGS) -c $(FORCE_ATLAS) -o $(FORCE_ATLAS_OUT)
	$(CC) $(FORCE_ATLAS_OUT) $(CONCURRENT) $(GRAPH_C) $(PRETRAITEMENT) -o $(LIBNATIVE_OUT) $(FLAGS)

	java -Djava.library.path=$(JOGL_DIR) --module-path $(JAVAFX_DIR) --add-modules $(JAVAFX_MODULES) $(JAVA_EXPORTS) $(GRAPH_JAVA_OUT)

clean:
	- rm -rf $(OUT_DIR)/*.o
	- rm -rf $(OUT_DIR)/*.h
	- rm -rf $(OUT_DIR)/*/*.class
	- rm -rf $(SRC_DIR)/*/*.o
	- rm -rf $(SRC_DIR)/*/*/*.o
	- rm -rf $(GRAPH_C_DIR)/*.o
	- rm -rf $(shell find out -mindepth 1 -type d)
	- rm -rf *.log
