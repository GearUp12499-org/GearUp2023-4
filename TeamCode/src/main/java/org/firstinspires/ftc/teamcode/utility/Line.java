package org.firstinspires.ftc.teamcode.utility;

public class Line {
    public final double slope;
    public final double yIntercept;

    private Line(double slope, double yIntercept) {
        this.slope = slope;
        this.yIntercept = yIntercept;
    }

    public double perpendicularSlope() {
        if (Double.isNaN(slope)) {
            return 0;
        }
        return -1 / slope;
    }

    public double yFor(double x) {
        return slope * x + yIntercept;
    }

    public double xFor(double y) {
        return (y - yIntercept) / slope;
    }

    public static Vector2 intersection(Line a, Line b) {
        // m1x + b1 = m2x + b2
        // (m1-m2)x = (b2-b1)
        // x = (b2-b1)/(m1-m2)
        double x = (b.yIntercept - a.yIntercept) / (a.slope - b.slope);
        double y = a.yFor(x);
        return new Vector2(x, y);
    }

    public static Line slopeIntercept(double slope, double yIntercept) {
        return new Line(slope, yIntercept);
    }

    public static Line pointSlope(Vector2 point, double slope) {
        return new Line(slope, point.y - slope * point.x);
    }

    public static Line pointPoint(Vector2 point1, Vector2 point2) {
        return pointSlope(point1, (point2.y - point1.y) / (point2.x - point1.x));
    }
}
