package frc.robot.subsystems.scoreMech;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ScoreMechSubsystem extends SubsystemBase {
    private ScoreMechIO io;
    private ScoreMechIOInputsAutoLogged inputs = new ScoreMechIOInputsAutoLogged();

    public ScoreMechSubsystem(ScoreMechIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Scoring Mechanism", inputs);
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
        io.setPower(0);
    }

    public Command spinUntilCoralSafe() {
        return runEnd(() -> io.setPower(1), () -> io.setPower(0)).until(this::coralIsSafe);
    }

    public Command spinManual(double power) {
        return runEnd(() -> io.setPower(power), () -> io.setPower(0));
    }

    public Command scoreCoral() {
        return runEnd(() -> io.setPower(coralIsSafe() ? 0.5 : 0), this::stopMotor);
    }
}
