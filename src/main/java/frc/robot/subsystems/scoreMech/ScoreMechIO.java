package frc.robot.subsystems.scoreMech;

import org.littletonrobotics.junction.AutoLog;

public interface ScoreMechIO {
    @AutoLog
    public class ScoreMechIOInputs {
        public double scoreVelocity = 0.0;
        public boolean innerSensorDetected = false;
        public boolean outerSensorDetected = false;
    }

    public default void updateInputs(ScoreMechIOInputs inputs) {
    }
    public default void setScoreVelocity(double scoreVelocity) {}

    public default void spinMotor(double scoreVelocity) {
    }
}
