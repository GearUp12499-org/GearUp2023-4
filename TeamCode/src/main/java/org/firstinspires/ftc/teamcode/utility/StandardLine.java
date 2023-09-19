package org.firstinspires.ftc.teamcode.utility;

public class StandardLine implements Line {
    private final double slope;
    private final double yIntercept;

    @Override
    public double perpendicularSlope() {
        return -1 / slope;
    }

    @Override
    public double yFor(double x) {
        return slope * x + yIntercept;
    }

    @Override
    public double xFor(double y) {
        return (y - yIntercept) / slope;
    }

    @Override
    public IntersectionSolution intersection(Line other) {
        if (other instanceof VerticalLine) {
            // Let the VerticalLine class handle this.
            return other.intersection(this);
        } else if (other instanceof StandardLine) {
            StandardLine other2 = (StandardLine) other;
            if (slope == other2.slope) {
                if (yIntercept == other2.yIntercept) {
                    return IntersectionSolution.sameLine();
                }
                return IntersectionSolution.none();
            }
            double x = (other2.yIntercept - yIntercept) / (slope - other2.slope);
            return IntersectionSolution.point(new Vector2(x, yFor(x)));
        } else {
            throw new IllegalArgumentException("Line type not supported");
        }
    }

    @Override
    public double slope() {
        return slope;
    }

    @Override
    public double yIntercept() {
        return yIntercept;
    }

    public StandardLine(double slope, double yIntercept) {
        this.slope = slope;
        this.yIntercept = yIntercept;
    }
}
