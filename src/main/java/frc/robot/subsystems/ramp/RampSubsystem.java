package frc.robot.subsystems.ramp;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RampSubsystem extends SubsystemBase {

    private RampIO io;
    private RampIOInputsAutoLogged inputs;

    public RampSubsystem(RampIO io) {
        this.io = io;
        inputs = new RampIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void RunRamp(double voltage) {
        io.setRollerVoltage(voltage);
    }
}
