package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command moveToPosition(double position) {
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
