CC = gcc
FLAGS = -Wall -g -lGL -lGLU -lglut -fPIC -lm -lc -shared
JNI_FLAGS = -I/usr/lib/jvm/java-21-openjdk-amd64/include -I/usr/lib/jvm/java-21-openjdk-amd64/include/linux -fPIC

SRC_DIR = src
OUT_DIR = out

JAVAFX_DIR = lib/javafx-sdk-23.0.2/lib
JAVAFX_MODULES = javafx.controls,javafx.fxml,javafx.swing

# Configuration de JOGL
JOGL_DIR = lib/jogl
JOGL_JARS = $(JOGL_DIR)/jogl-all.jar:$(JOGL_DIR)/gluegen-rt.jar
JOGL_NATIVE_LIBS = $(JOGL_DIR)/jogl-all-natives-linux-amd64.jar:$(JOGL_DIR)/gluegen-rt-natives-linux-amd64.jar
CLASSPATH = $(JOGL_JARS):$(JOGL_NATIVE_LIBS):$(OUT_DIR)

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

# Compilation Java
GRAPH_JAVA = $(SRC_DIR)/*/*.java
JAVAC_FLAGS = --module-path $(JAVAFX_DIR) --add-modules $(JAVAFX_MODULES) -cp $(CLASSPATH) -d $(OUT_DIR)

GRAPH_JAVA_MAIN = graph.Graph

# C sources
GRAPH_C_DIR  = $(SRC_DIR)/c/c_graph
GRAPH_C_OBJ  = graph.o cluster.o communities.o
GRAPH_C      = $(patsubst %, $(GRAPH_C_DIR)/%, $(GRAPH_C_OBJ))

DEBUG_DIR  = $(SRC_DIR)/c/debug
DEBUG_OBJ  = debug_time.o
DEBUG      = $(patsubst %, $(DEBUG_DIR)/%, $(DEBUG_OBJ))

CONCURRENT_DIR  = $(SRC_DIR)/c/concurrent
CONCURRENT_OBJ  = Pool.o Barrier.o Tools.o
CONCURRENT      = $(patsubst %, $(CONCURRENT_DIR)/%, $(CONCURRENT_OBJ))

PRETRAITEMENT_DIR = $(SRC_DIR)/c/pretraitement
PRETRAITEMENT_OBJ = data.o similarity.o
PRETRAITEMENT     = $(patsubst %, $(PRETRAITEMENT_DIR)/%, $(PRETRAITEMENT_OBJ))

FORCE_ATLAS = $(SRC_DIR)/c/forceatlasV4_CSV.c
FORCE_ATLAS_OUT = $(OUT_DIR)/forceatlas.o
LIBNATIVE_OUT = $(OUT_DIR)/libnative.so

DIR_SAMPLES = samples
DIR_SAMPLE = $(DIR_SAMPLES)/iris.csv

# Compilation C
$(DEBUG_DIR)/%.o: $(DEBUG_DIR)/%.c $(DEBUG_DIR)/%.h
	$(CC) -c -o $@ $< $(FLAGS) $(JNI_FLAGS)

$(CONCURRENT_DIR)/%.o: $(CONCURRENT_DIR)/%.c $(CONCURRENT_DIR)/%.h
	$(CC) -c -o $@ $< $(FLAGS) $(JNI_FLAGS)

$(GRAPH_C_DIR)/%.o: $(GRAPH_C_DIR)/%.c $(GRAPH_C_DIR)/%.h
	$(CC) -c -o $@ $< $(FLAGS) $(JNI_FLAGS)

$(PRETRAITEMENT_DIR)/%.o: $(PRETRAITEMENT_DIR)/%.c $(PRETRAITEMENT_DIR)/%.h
	$(CC) -c -o $@ $< $(FLAGS) $(JNI_FLAGS)

# Règle principale
all: $(CONCURRENT) $(GRAPH_C) $(PRETRAITEMENT)
	# Compilation Java avec génération des headers JNI
	javac $(JAVAC_FLAGS) $(GRAPH_JAVA)
	javac $(JAVAC_FLAGS) -h $(OUT_DIR) $(GRAPH_JAVA)

	# Compilation du code natif C en bibliothèque partagée
	$(CC) $(JNI_FLAGS) -c $(FORCE_ATLAS) -o $(FORCE_ATLAS_OUT)
	$(CC) $(FORCE_ATLAS_OUT) $(CONCURRENT) $(GRAPH_C) $(PRETRAITEMENT) -o $(LIBNATIVE_OUT) $(FLAGS)

	# Exécution de l’application Java
	java \
		-Djava.library.path=$(JOGL_DIR) \
		--module-path $(JAVAFX_DIR) \
		--add-modules $(JAVAFX_MODULES) \
		$(JAVA_EXPORTS) \
		-cp $(CLASSPATH):$(OUT_DIR) \
		$(GRAPH_JAVA_MAIN) $(DIR_SAMPLE)

# Nettoyage
clean:
	- rm -rf $(OUT_DIR)/*.o
	- rm -rf $(OUT_DIR)/*.h
	- rm -rf $(OUT_DIR)/*/*.class
	- rm -rf $(SRC_DIR)/*/*.o
	- rm -rf $(SRC_DIR)/*/*/*.o
	- rm -rf $(GRAPH_C_DIR)/*.o
	- rm -rf $(shell find out -mindepth 1 -type d)
