package frc.robot.subsystems.ramp;

import static edu.wpi.first.wpilibj2.command.Commands.runEnd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public boolean outerRollerDetection() {
        return inputs.outerSensorDetected;
    }

    public boolean innerRollerDetection() {
        return inputs.innerSensorDetected;
    }

    public Command setRoller(double voltage) {
        return runEnd(
                        () -> {
                            io.setRollerVoltage(voltage);
                        },
                        () -> {
                            io.setRollerVoltage(0);
                        })
                .until(this::innerRollerDetection);
    }
}
