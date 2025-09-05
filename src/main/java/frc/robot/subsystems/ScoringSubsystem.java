package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.ScoreConfigs.scoringMotorSpeed;

public class ScoringSubsystem extends SubsystemBase {
    private final TalonFX scoringMotor;
    private final CANrange insideCANRange;
    private final CANrange outsideCANRange;

    public ScoringSubsystem() {
        scoringMotor = new TalonFX(ScoreConfigs.scoringMotorID);
        insideCANRange = new CANrange(ScoreConfigs.canRangeStartID);
        outsideCANRange = new CANrange(ScoreConfigs.canRangeEndID);
    }

    public boolean coralInMechanism() {
        return (insideCANRange.getIsDetected().getValue() && outsideCANRange.getIsDetected().getValue());
    }

    public boolean coralIsSafe() {
        return (outsideCANRange.getIsDetected().getValue() && !insideCANRange.getIsDetected().getValue());
    }

    private void setScoringSpeed(double scoringMotorSpeed) { scoringMotor.set(scoringMotorSpeed); }

    private void stopMotor() {scoringMotor.stopMotor();}

    public Command spinRollers(double scoringMotorSpeed) {return startEnd(() -> setScoringSpeed(coralInMechanism() ? scoringMotorSpeed : 0), this::stopMotor); }
}