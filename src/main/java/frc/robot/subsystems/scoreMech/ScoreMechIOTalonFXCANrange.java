package frc.robot.subsystems.scoreMech;

import static frc.robot.constants.Constants.motorCANBus;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

public class ScoreMechIOTalonFXCANrange implements ScoreMechIO {
    private TalonFX scoreMotor;
    private CANrange innerCANrange;
    private CANrange outerCANrange;

    public ScoreMechIOTalonFXCANrange() {
        scoreMotor = new TalonFX(ScoreConfigs.scoreMotorID, motorCANBus);
        innerCANrange = new CANrange(ScoreConfigs.innerCANrangeID, motorCANBus);
        outerCANrange = new CANrange(ScoreConfigs.outerCANrangeID, motorCANBus);
    }

    public void updateInputs(ScoreMechIOInputs inputs) {
        inputs.scoreVelocity = scoreMotor.getVelocity().getValueAsDouble();
        inputs.innerSensorDetected = innerCANrange.getIsDetected().getValue();
        inputs.outerSensorDetected = outerCANrange.getIsDetected().getValue();
    }

    public void setScoreVelocity(double scoreVelocity) {
        scoreMotor.set(scoreVelocity);
    }
}
