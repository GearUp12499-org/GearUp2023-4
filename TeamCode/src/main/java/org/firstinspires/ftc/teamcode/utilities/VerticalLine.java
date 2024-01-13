package org.firstinspires.ftc.teamcode.utilities;

/**
 * A special case of a line that is vertical.
 * This is a special case because it has no slope.
 */
public class VerticalLine implements Line {
    private final double xIntercept;

    VerticalLine(double xIntercept) {
        this.xIntercept = xIntercept;
    }

    @Override
    public double perpendicularSlope() {
        return 0;
    }

    @Override
    public double yFor(double x) {
        if (x == xIntercept) return 0;
        return Double.NaN;
    }

    @Override
    public double xFor(double y) {
        return xIntercept;
    }

    @Override
    public IntersectionSolution intersection(Line other) {
        if (other instanceof VerticalLine) {
            if (xIntercept == ((VerticalLine) other).xIntercept) {
                return IntersectionSolution.sameLine();
            }
            return IntersectionSolution.none();
        }
        return IntersectionSolution.point(new Vector2(xIntercept, other.yFor(xIntercept)));
    }

    @Override
    public double slope() {
        return Double.NaN;
    }

    @Override
    public double yIntercept() {
        if (xIntercept == 0) return 0;
        return Double.NaN;
    }
}
