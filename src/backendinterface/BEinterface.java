package backendinterface;

/**
 * Pour pouvoir utiliser l'interface il faut faire:
 * Il faut changer les JNI_FLAGS pour que ça fonctionne sur vos PC
 * Changer le chemin vers libnative dans System.load juste en-dessous
 * Ensuite, make all devrait tout compiler
 * Pour executer, l'exemple dans le main, je faisais : java -cp . backendinterface.BEinterface ../samples/predicancerNUadd9239.csv
 */
public class BEinterface {

    static {
        String libnative = System.getProperty("user.dir") + "/out/libnative.so";
        System.load(libnative);
    }


    /**
     * Updates the graph
     */
    public native void updatePositions();

    /**
     * returns One of the array used in the display function
     * @return int array representing connections between points
     */
    public native int[] getCommunitites();

    /**
     * returns One of the array used in the display function
     * @return array representing the colors of the clusters
     */
    public native float[][] getClusterColors();

    /**
     * returns One of the array used in the display function
     * @return array representing the edges of the graph
     */
    public native Edge[] getEdges();

    /** ATTENTION ce n'est pas la classe de Java.awt, je n'arrivais pas à linker la librairie
     * returns One of the array used in the display function
     * @return array representing the points of the graph
     */
    public native Point[] getPositions();

    /**
     * starts the program with filename as argument
     */
    public native void startsProgram(String filename);

    /**
     * free memory that allocated on the heap
     */
    public native void freeAllocatedMemory();

    public static void main(String[] args)
    {

        //System.out.println(System.getProperty("user.dir"));

        BEinterface bei = new BEinterface();

        bei.startsProgram(args[0]);

        float[][] color = bei.getClusterColors();
        int[] communitites = bei.getCommunitites();
        Edge[] edges = bei.getEdges();
        Point[] positions = bei.getPositions();

        System.out.println(positions[positions.length - 1].x + " " + positions[positions.length - 1].y);

        bei.updatePositions();

        positions = bei.getPositions();
        System.out.println(positions[positions.length - 1].x + " " + positions[positions.length - 1].y);

        bei.freeAllocatedMemory();
    }
}
