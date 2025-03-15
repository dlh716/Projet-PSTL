package graph;

public class Community {
    private final int id;
    private final double color_r;
    private final double color_g;
    private final double color_b;



    public Community(int id, double r, double g, double b) {
        this.id = id;
        color_r = r;
        color_g = g;
        color_b = b;
    }


    public int getId() {
        return id;
    }

    public double getR() {
        return color_r;
    }
    public double getG() {
        return color_g;
    }
    public double getB() {
        return color_b;
    }

    public String toString() {
        return "Communauté " + id + " (R : " + String.format("%.2f", color_r) + ", G : " + String.format("%.2f", color_g) + ", B : " + String.format("%.2f", color_b) + ")";
    }

}
