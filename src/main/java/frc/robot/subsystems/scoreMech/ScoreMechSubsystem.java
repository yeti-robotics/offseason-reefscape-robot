package frc.robot.subsystems.scoreMech;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoreMechSubsystem extends SubsystemBase {
    private ScoreMechIO io;
    private ScoreMechIOInputsAutoLogged inputs;

    public ScoreMechSubsystem(ScoreMechIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public boolean innerSensorDetected() {
        return inputs.innerSensorDetected;
    }

    public boolean outerSensorDetected() {
        return inputs.outerSensorDetected;
    }

    public double getScoreVelocity() {
        return inputs.scoreVelocity;
    }

    public boolean coralIsSafe() {
        return !innerSensorDetected() && outerSensorDetected();
    }

    private void stopMotor() {
        io.setScoreVelocity(0);
    }

    public Command scoreCoral() {
        return runEnd(() -> io.setScoreVelocity(coralIsSafe() ? getScoreVelocity() : 0), this::stopMotor);
    }
}