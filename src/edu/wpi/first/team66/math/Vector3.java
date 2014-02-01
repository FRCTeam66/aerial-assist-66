package edu.wpi.first.team66.math;

public class Vector3 {
    public final double x;
    public final double y;
    public final double z;
    
    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    
    public Vector3 add(Vector3 b) {
        return new Vector3(x + b.x, y + b.y, z + b.z);
    }
    
    public Vector3 subtract(Vector3 b) {
        return new Vector3(x - b.x, y - b.y, z - b.z);
    }
    
    public Vector3 scale(double s) {
        return new Vector3(s * x, s * y, s * z);
    }
    
    public double dot(Vector3 b) {
        return x * b.x + y * b.y + z * b.z;
    }
}
