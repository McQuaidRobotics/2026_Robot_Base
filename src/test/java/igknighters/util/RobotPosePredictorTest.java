package igknighters.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class RobotPosePredictorTest {
    @Test
    void indexOfMaxAndMinReturnFirstMatch() {
        double[] values = {3.0, 7.0, 1.0, 7.0, 1.0};
        assertEquals(1, RobotPosePredictor.indexOfMax(values));
        assertEquals(2, RobotPosePredictor.indexOfMin(values));
        assertEquals(0, RobotPosePredictor.indexOfMax(new double[] {0, 0, 0}));
    }
}
