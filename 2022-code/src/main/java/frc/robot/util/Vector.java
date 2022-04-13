package frc.robot.util;

public class Vector {

    private double x;
    private double y;
    
    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getNorm() {
        return Math.sqrt(x*x + y*y);
    }

    public double dotProdWith(Vector w) {
        return x * w.getX() + y * w.getY();
    }

    public double arg(Vector w) {
        double term = this.dotProdWith(w) / (getNorm() * w.getNorm());
        return Math.acos(term);
    }

    public Vector addWith(Vector w) {
        return new Vector(x + w.getX(), y + w.getY());
    }

    public Vector inverse() {
        return new Vector(-x, -y);
    }

    public static double argFromLOC(double adj1, double adj2, double opp) {
        double term = (adj1*adj1 + adj2*adj2 - opp*opp) / (2*adj1*adj2);
        return Math.acos(term);
    }
}
