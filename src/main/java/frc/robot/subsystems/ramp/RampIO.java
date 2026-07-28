package frc.robot.subsystems.ramp;

import org.littletonrobotics.junction.AutoLog;

public interface RampIO {
    @AutoLog
    public static class RampIOInputs {
        public double rollerVelocityRPM = 1;
        public double rollerVoltage = 1;

        public double rangeInMeters = 1.0;
        public boolean outerSensorDetected = true;
        public boolean innerSensorDetected = true;
    }

    public default void updateInputs(RampIOInputs inputs) {}

    public default void setRollerDuty(double volts) {}
}
