package frc.robot.subsystems.scoreMech;

import com.ctre.phoenix6.hardware.CANrange;

public interface ScoreMechIOTalonFxCANRange {
    private final TalonFx scoreMotor;
    privete final CANrange insideCANRange;
    private final CANrange outsideCANRange;
    public class ScoreMechIOTalonFxCANRange{
        scoreMotor = new TalonFX(ScoreConfigs.scoringMotorID, motorCANBus);
        insideCANRange = new CANrange(ScoreConfigs.canRangeStartID, motorCANBus);
        outsideCANRange = new CANrange(ScoreConfigs.canRangeEndID, motorCANBus

    }

}
