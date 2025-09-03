package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoringSubsystem extends SubsystemBase {
    private final TalonFX scoringMotor= new TalonFX(ScoreConfigs.scoringMotorID);
    private final CANrange insideCANRange = new CANrange(ScoreConfigs.canRangeStartID);
    private final CANrange outsideCANRange = new CANrange(ScoreConfigs.canRangeEndID);

    public ScoringSubsystem() {
    }
}

