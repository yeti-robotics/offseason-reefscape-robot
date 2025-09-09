package frc.robot.subsystems.scoreMech;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ScoreConfigs;

import static frc.robot.constants.Constants.motorCANBus;

public class ScoringSubsystem extends SubsystemBase {
    private final TalonFX scoringMotor;
    private final CANrange insideCANRange;
    private final CANrange outsideCANRange;

    public ScoringSubsystem() {
        scoringMotor = new TalonFX(ScoreConfigs.scoringMotorID, motorCANBus);
        insideCANRange = new CANrange(ScoreConfigs.canRangeStartID, motorCANBus);
        outsideCANRange = new CANrange(ScoreConfigs.canRangeEndID, motorCANBus);
    }

    public boolean coralIsSafe() {
        return outsideCANRange.getIsDetected().getValue() && !insideCANRange.getIsDetected().getValue();
    }

    private void setScoringSpeed(double scoringMotorSpeed) {scoringMotor.set(scoringMotorSpeed); }

    private void stopMotor() {scoringMotor.stopMotor();}

    public Command spinRollers(double scoringMotorSpeed) {return runEnd(() -> setScoringSpeed(coralIsSafe() ? 0 : scoringMotorSpeed), this::stopMotor);}
}