package org.firstinspires.ftc.teamcode.utilities;

public interface Line {
    double perpendicularSlope();
    double yFor(double x);
    double xFor(double y);
    IntersectionSolution intersection(Line other);

    double slope();
    double yIntercept();

    static Line slopeIntercept(double slope, double yIntercept) {
        if (Double.isNaN(slope)) return new VerticalLine(0);
        return new StandardLine(slope, yIntercept);
    }

    static Line pointSlope(Vector2 point, double slope) {
        if (Double.isNaN(slope)) return new VerticalLine(point.x);
        return new StandardLine(slope, point.y - slope * point.x);
    }

    static Line pointPoint(Vector2 point1, Vector2 point2) {
        if (point1.x == point2.x) {
            return new VerticalLine(point1.x);
        }
        return pointSlope(point1, (point2.y - point1.y) / (point2.x - point1.x));
    }

    class IntersectionSolution {
        enum Type {
            NONE,
            POINT,
            SAME_LINE
        }
        public final Type type;
        public final Vector2 pointValue;
        private IntersectionSolution(Type type, Vector2 pointValue) {
            this.type = type;
            this.pointValue = pointValue;
        }
        public static IntersectionSolution point(Vector2 point) {
            return new IntersectionSolution(Type.POINT, point);
        }
        public static IntersectionSolution sameLine() {
            return new IntersectionSolution(Type.SAME_LINE, null);
        }
        public static IntersectionSolution none() {
            return new IntersectionSolution(Type.NONE, null);
        }
    }
}
