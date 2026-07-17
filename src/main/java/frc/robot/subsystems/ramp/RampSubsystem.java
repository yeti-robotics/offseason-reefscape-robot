package frc.robot.subsystems.ramp;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;

public class RampSubsystem extends SubsystemBase {

    private RampIO io;
    private RampIOInputsAutoLogged inputs = new RampIOInputsAutoLogged();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public RampSubsystem(RampIO io) {
        this.io = io;
    }

    public boolean outerRampDetection() {
        return inputs.outerSensorDetected;
    }

    public boolean innerRampDetection() {
        return inputs.innerSensorDetected;
    }

    public Command setRoller(double power) {
        return runEnd(
                () -> {
                    io.setRollerDuty(power);
                },
                () -> {
                    io.setRollerDuty(0);
                });
    }
}
