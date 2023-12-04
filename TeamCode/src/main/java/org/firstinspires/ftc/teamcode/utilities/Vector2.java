package org.firstinspires.ftc.teamcode.utilities;

import androidx.annotation.NonNull;

import java.util.Locale;

public class Vector2 {
    public static Vector2 origin = new Vector2(0, 0);
    public final double x;
    public final double y;

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2 scale(double scalar) {
        return new Vector2(x * scalar, y * scalar);
    }

    public Vector2 scale(double xScale, double yScale) {
        return new Vector2(x * xScale, y * yScale);
    }

    public Vector2 add(Vector2 other) {
        return new Vector2(x + other.x, y + other.y);
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.US, "(%.3f, %.3f)", x, y);
    }
}
