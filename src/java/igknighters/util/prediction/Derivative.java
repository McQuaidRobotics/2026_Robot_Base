
package igknighters.util.prediction;

public class Derivative {

    public double makeDerivative(Point[] points) {
        if (points == null || points.length < 2) {
            return Double.NaN;
        }

        double total = 0.0;
        int count = 0;

        for (int i = 1; i < points.length; i++) {
            Point firstPoint = points[i - 1];
            Point secondPoint = points[i];

            double dx = secondPoint.x - firstPoint.x;

            // Avoid division by zero
            if (dx == 0) {
                continue;
            }

            double slope =
                (secondPoint.y - firstPoint.y) / dx;

            total += slope;
            count++;
        }

        return count > 0 ? total / count : 0.0;
    }
}
