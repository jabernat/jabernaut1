package org.firstinspires.ftc.teamcode;




public class Vec2D {
    private double x;
    private double y;

    public Vec2D() {
        this(0.0, 0.0);
    }
    public Vec2D(final double x, final double y) {
        this.x = x;
        this.y = y;
    }
    public Vec2D(final Vec2D other) {
        x = other.x;
        y = other.y;
    }

    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }

    public double getMagnitude() {
        return Math.hypot(x, y);
    }
    public void changeMagnitude(final double magnitude) {
        final double magnitudeOld = getMagnitude();
        if (magnitudeOld == 0.0) {
            return;
        }

        // Normalize and apply new magnitude
        multiply(magnitude / magnitudeOld);
    }

    public double dotProduct(final Vec2D other) {
        return x * other.x + y * other.y;
    }

    public void normalize() {
        final double magnitude = getMagnitude();
        if (magnitude == 0.0) {  // Undefined
            x = Double.NaN;
            y = Double.NaN;
        } else {
            x /= magnitude;
            y /= magnitude;
        }
    }
    public Vec2D normalized() {
        final double magnitude = getMagnitude();
        if (magnitude == 0.0) {  // Undefined
            return new Vec2D(Double.NaN, Double.NaN);
        } else {
            return new Vec2D(x / magnitude, y / magnitude);
        }
    }

    public void negate() {
        x = -x;
        y = -y;
    }
    public Vec2D negated() {
        return new Vec2D(-x, -y);
    }

    public void add(final Vec2D other) {
        x += other.x;
        y += other.y;
    }
    public Vec2D added(final Vec2D other) {
        return new Vec2D(x + other.x, y + other.y);
    }

    public void subtract(final Vec2D other) {
        x -= other.x;
        y -= other.y;
    }
    public Vec2D subtracted(final Vec2D other) {
        return new Vec2D(x - other.x, y - other.y);
    }

    public void multiply(final double coefficient) {
        x *= coefficient;
        y *= coefficient;
    }
    public Vec2D multiplied(final double coefficient) {
        return new Vec2D(x * coefficient, y * coefficient);
    }

    public void divide(final double coefficient) {
        x /= coefficient;
        y /= coefficient;
    }
    public Vec2D divided(final double coefficient) {
        return new Vec2D(x / coefficient, y / coefficient);
    }
}
