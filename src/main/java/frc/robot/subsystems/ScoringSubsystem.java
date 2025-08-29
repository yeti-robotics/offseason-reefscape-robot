package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoringSubsystem extends SubsystemBase {
    private final TalonFX scoringMotor= new TalonFX(ScoreConfigs.scoringMotorID); ;
    public ScoringSubsystem() {
    }
}

