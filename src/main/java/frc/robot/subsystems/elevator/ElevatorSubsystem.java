package frc.robot.subsystems.elevator;

import org.wpilib.units.measure.Angle;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
        new Trigger(() -> inputs.isAtBottom).onTrue(runOnce(io::zeroPosition).andThen(io::neutralizeElevator));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public Command moveToPosition(Angle position) {
        return runOnce(() -> io.moveToPosition(position));
    }

    public boolean isAtBottom() {
        return inputs.isAtBottom;
    }

    public double getCurrentPosition() {
        return inputs.positionRotation;
    }

    public double getTargetPosition() {
        return inputs.targetPositionRotation;
    }
}
