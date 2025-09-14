package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConfig.*;
import static frc.robot.subsystems.elevator.ElevatorIOTalonFXCANRange.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void moveToPosition(double position) {
        io.moveToPosition(position);
    }

    public boolean isAtBottom() {
        return inputs.isAtBottom;
    }
}
