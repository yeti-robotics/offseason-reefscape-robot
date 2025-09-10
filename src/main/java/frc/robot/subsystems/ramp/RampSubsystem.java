package frc.robot.subsystems.ramp;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RampSubsystem extends SubsystemBase {

    private RampIO io;
    private RampIOInputsAutoLogged inputs;

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public RampSubsystem(RampIO io) {
        this.io = io;
        inputs = new RampIOInputsAutoLogged();
    }

    public void runRamp(double voltage) {
        io.setRollerVoltage(voltage);
    }

    public boolean outerRollerDetection() {
        return inputs.outerSensorDetected;
    }

    public boolean innerRollerDetection() {
        return inputs.innerSensorDetected;
    }
}
