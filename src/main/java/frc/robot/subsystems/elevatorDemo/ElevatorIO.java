package frc.robot.subsystems.elevatorDemo;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double positionRotation = 0.0;
        public double velocityRPM = 0.0;
        public boolean isAtBottom = false;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void moveToPosition(double position) {}
}
