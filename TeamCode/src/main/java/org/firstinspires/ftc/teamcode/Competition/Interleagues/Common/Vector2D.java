package org.firstinspires.ftc.teamcode.Competition.Interleagues.Common;

/**
 * Represents an immutable 2-dimensional vector.
 * This class is immutable.
 */
public final class Vector2D implements Cloneable {

    private double x;
    private double y;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }




    /**
     * Returns a new vector with the specified angle and magnitude.
     */
    public static Vector2D fromAngleMagnitude(double theta, double magnitude) {
        return new Vector2D(Math.cos(theta) * magnitude, Math.sin(theta) * magnitude);
    }

    public static Vector2D up() {
        return new Vector2D(0, 1);
    }

    public static Vector2D right() {
        return new Vector2D(1, 0);
    }

    public static Vector2D zero() {
        return new Vector2D(0, 0);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }

    public Vector2D normalize() {
        double magnitude = magnitude();
        return new Vector2D(x / magnitude, y / magnitude);
    }

    public Vector2D add(Vector2D vector) {
        return new Vector2D(x + vector.x, y + vector.y);
    }

    public Vector2D subtract(Vector2D vector) {
        return new Vector2D(x - vector.x, y - vector.y);
    }

    public Vector2D multiply(double scalar) {
        return new Vector2D(x * scalar, y * scalar);
    }

    /**
     * Returns a new vector that is a transformation of this vector rotated counterclockwise around
     * the origin.
     */
    public Vector2D rotate(double radians) {
        double x2 = Math.cos(radians) * this.x - Math.sin(radians) * this.y;
        double y2 = Math.sin(radians) * this.x + Math.cos(radians) * this.y;
        return new Vector2D(x2, y2);
    }

    public double dotProduct(Vector2D other) {
        return this.x * other.x + this.y * other.y;
    }

    /**
     * @return the angle in radians
     */
    public double angleBetween(Vector2D other) {
        return Math.acos(this.dotProduct(other) / (this.magnitude() * other.magnitude()));
    }

    public boolean isZero() {
        return x == 0.0 && y == 0.0;
    }

    /**
     * @return the angle in radians.
     */
    public double getDirection() {
        return Math.atan2(y, x);
    }

    @Override
    public String toString() {
        return String.format("[%.3f, %.3f]", x, y);
    }

    @Override
    public Vector2D clone() throws CloneNotSupportedException {
        Vector2D newVec = (Vector2D)super.clone();
        newVec.x = this.x;
        newVec.y = this.y;
        return newVec;
    }

}