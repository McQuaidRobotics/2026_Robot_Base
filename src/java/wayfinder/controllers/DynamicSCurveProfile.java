package wayfinder.controllers;

import wayfinder.controllers.Types.State;

public final class DynamicSCurveProfile {
    private DynamicSCurveProfile() {
        throw new UnsupportedOperationException(
                "DynamicSCurveProfile is a utility class and cannot be instantiated");
    }

    private static boolean shouldFlipAcceleration(double initialPosition, double goalPosition) {
        return initialPosition > goalPosition;
    }

    private static double direct(double number, boolean forward) {
        double direction = forward ? -1.0 : 1.0;
        return number * direction;
    }

    public static State calculate(
            double period,
            double currentPos,
            double currentVel,
            double currentAcc,
            double goalPos,
            double goalVel,
            double goalAcc,
            double maxVel,
            double maxAccel,
            double maxJerk) {

        // 1. Handle direction flipping
        boolean direction = shouldFlipAcceleration(currentPos, goalPos);
        currentPos = direct(currentPos, direction);
        currentVel = direct(currentVel, direction);
        currentAcc = direct(currentAcc, direction);
        goalPos = direct(goalPos, direction);
        goalVel = direct(goalVel, direction);
        goalAcc = direct(goalAcc, direction);

        // 2. Calculate Phase Durations (The 7 phases of an S-Curve)
        double tJerk = maxAccel / maxJerk;
        double deltaVRamp = maxAccel * tJerk;

        double tAccConstant;
        if (maxVel - currentVel > deltaVRamp) {
            tAccConstant = (maxVel - currentVel - deltaVRamp) / maxAccel;
        } else {
            tJerk = Math.sqrt((maxVel - currentVel) / maxJerk);
            tAccConstant = 0;
            maxAccel = tJerk * maxJerk;
        }

        double t1 = tJerk;
        double t2 = t1 + tAccConstant;
        double t3 = t2 + tJerk;

        double distAcc =
                currentVel * t3
                        + 0.5 * maxAccel * tJerk * t3
                        + (tAccConstant > 0 ? (0.5 * maxAccel * tAccConstant * (t2 + t3)) : 0);

        double totalDist = goalPos - currentPos;
        double distDecel = distAcc;
        double distCruise = totalDist - distAcc - distDecel;

        double tCruise = 0;
        if (distCruise > 0) {
            tCruise = distCruise / maxVel;
        } else {
            distCruise = 0;
            double scale = Math.sqrt(totalDist / (distAcc + distDecel));
            t1 *= scale;
            t2 *= scale;
            t3 *= scale;
            tJerk = t1;
            tAccConstant = t2 - t1;
            t3 = t2 + tJerk;
            maxAccel = tJerk * maxJerk;
            maxVel = currentVel + maxAccel * tJerk + maxAccel * tAccConstant;
        }

        // Map out the final 7-phase execution timestamps
        double endJerkUp = t1;
        double endConstantAcc = t2;
        double endJerkDown = t3;
        double endCruise = endJerkDown + tCruise;
        double endDecelJerkUp = endCruise + t1;
        double endConstantDec = endDecelJerkUp + (t2 - t1);
        double endDecelJerkDn = endConstantDec + t1;

        double resultPos = currentPos;
        double resultVel = currentVel;
        double resultAcc = currentAcc;

        // Precompute state boundaries to ensure continuous position accumulation
        double v1 = currentVel + 0.5 * maxJerk * endJerkUp * endJerkUp;
        double p1 =
                currentPos
                        + currentVel * endJerkUp
                        + (1.0 / 6.0) * maxJerk * endJerkUp * endJerkUp * endJerkUp;

        double tAcc = endConstantAcc - endJerkUp;
        double v2 = v1 + maxAccel * tAcc;
        double p2 = p1 + v1 * tAcc + 0.5 * maxAccel * tAcc * tAcc;

        double tJerkDown = endJerkDown - endConstantAcc;
        double p3 =
                p2
                        + v2 * tJerkDown
                        + 0.5 * maxAccel * tJerkDown * tJerkDown
                        - (1.0 / 6.0) * maxJerk * tJerkDown * tJerkDown * tJerkDown;

        // 3. Analytical Time-Slice Interpolation Block
        if (period < endJerkUp) {
            resultAcc += period * maxJerk;
            resultVel += currentAcc * period + 0.5 * maxJerk * period * period;
            resultPos +=
                    currentVel * period
                            + 0.5 * currentAcc * period * period
                            + (1.0 / 6.0) * maxJerk * period * period * period;
        } else if (period < endConstantAcc) {
            double dt = period - endJerkUp;
            resultAcc = maxAccel;
            resultVel = v1 + maxAccel * dt;
            resultPos = p1 + v1 * dt + 0.5 * maxAccel * dt * dt;
        } else if (period < endJerkDown) {
            double dt = period - endConstantAcc;
            resultAcc = maxAccel - dt * maxJerk;
            resultVel = v2 + maxAccel * dt - 0.5 * maxJerk * dt * dt;
            resultPos =
                    p2 + v2 * dt + 0.5 * maxAccel * dt * dt - (1.0 / 6.0) * maxJerk * dt * dt * dt;
        } else if (period < endCruise) {
            double dt = period - endJerkDown;
            resultAcc = 0;
            resultVel = maxVel;
            resultPos = p3 + maxVel * dt;
        } else if (period <= endDecelJerkDn) {
            double timeLeft = endDecelJerkDn - period;
            if (timeLeft < t1) {
                resultAcc = -timeLeft * maxJerk;
                resultVel = goalVel + 0.5 * maxJerk * timeLeft * timeLeft;
                resultPos =
                        goalPos
                                - (goalVel * timeLeft
                                        + (1.0 / 6.0) * maxJerk * timeLeft * timeLeft * timeLeft);
            } else {
                resultAcc = -maxAccel;
                resultVel = goalVel + maxAccel * timeLeft;
                resultPos = goalPos - (goalVel * timeLeft + 0.5 * maxAccel * timeLeft * timeLeft);
            }
        } else {
            resultPos = goalPos;
            resultVel = goalVel;
            resultAcc = goalAcc;
        }

        return new State(direct(resultPos, direction), direct(resultVel, direction));
    }
}
