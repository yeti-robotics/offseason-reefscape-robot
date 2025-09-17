package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs;

    private final MotionMagicTorqueCurrentFOC magicRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(0);

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
