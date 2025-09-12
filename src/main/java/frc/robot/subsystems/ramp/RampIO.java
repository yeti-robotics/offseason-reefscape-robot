package frc.robot.subsystems.ramp;

import org.littletonrobotics.junction.AutoLog;

public interface RampIO {
    @AutoLog
    public static class RampIOInputs {
        public double rollerVelocityRPM = 0;
        public double rollerVoltage = 0;

        public double rangeInMeters = 0.0;
        public boolean outerSensorDetected = false;
        public boolean innerSensorDetected = false;
    }

    public default void updateInputs(RampIOInputs inputs) {}

    public default void setRollerVoltage(double volts) {}
}
