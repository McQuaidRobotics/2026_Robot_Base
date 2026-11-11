package igknighters.util.prediction;

import static edu.wpi.first.units.Units.Second;

import java.util.ArrayList;

public class Derivative {

    public double makeDerivative(ArrayList<Sample> points) {
        if (points == null || points.size() < 2) {
            return Double.NaN;
        }

        double total = 0.0;
        int count = 0;
        ArrayList<Double> slopes = new ArrayList<>();

        for (int i = 1; i < points.size(); i++) {
            Sample firstPoint = points.get(i - 1);
            Sample secondPoint = points.get(i);

            double dx = secondPoint.x - firstPoint.x;
            double dt = secondPoint.t.in(Second) - firstPoint.t.in(Second);

            // Avoid division by zero
            if (dx == 0) {
                continue;
            }

            double slope = dx / dt;

            slopes.add(slope);
            count++;
        }
        for (int i = 1; i < slopes.size(); i++) {
            Double firstSlope = slopes.get(i - 1);
            Double secondSlope = slopes.get(i);

            double dslope = secondSlope - firstSlope;
            total += dslope;
        }

        return count > 0 || !slopes.isEmpty() ? total / count + slopes.get(slopes.size() - 1) : 0.0;
    }
}
