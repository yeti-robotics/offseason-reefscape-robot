package frc.robot.subsystems.scoreMech;

import static frc.robot.constants.Constants.motorCANBus;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.UpdateModeValue;

public class ScoreMechIOTalonFXCANrange implements ScoreMechIO {
    private TalonFX scoreMotor;
    private static CANrange innerCANrange;
    private static CANrange outerCANrange;

    public ScoreMechIOTalonFXCANrange() {
        scoreMotor = new TalonFX(ScoreConfigs.scoreMotorID, motorCANBus);
        innerCANrange = new CANrange(ScoreConfigs.innerCANrangeID, motorCANBus);
        outerCANrange = new CANrange(ScoreConfigs.outerCANrangeID, motorCANBus);
        applyConfigs();
    }

    public void updateInputs(ScoreMechIOInputs inputs) {
        inputs.scoreVelocity = scoreMotor.getVelocity().getValueAsDouble();
        inputs.innerSensorDetected = innerCANrange.getIsDetected().getValue();
        inputs.outerSensorDetected = outerCANrange.getIsDetected().getValue();
    }

    public static void applyConfigs() {
        var CANrangeConfig = new CANrangeConfiguration();
        CANrangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        CANrangeConfig.FovParams.FOVCenterX = 11.8;
        CANrangeConfig.FovParams.FOVCenterY = 11.8;
        CANrangeConfig.FovParams.FOVRangeX = 6.75;
        CANrangeConfig.FovParams.FOVRangeY = 6.75;
        CANrangeConfig.ProximityParams.ProximityThreshold = 0.17;
        CANrangeConfig.ProximityParams.ProximityHysteresis = 0.02;
        innerCANrange.getConfigurator().apply(CANrangeConfig);

        CANrangeConfig.FovParams.FOVCenterX = 0;
        CANrangeConfig.FovParams.FOVCenterY = 0;
        outerCANrange.getConfigurator().apply(CANrangeConfig);
    }

    @Override
    public void setPower(double scorePower) {
        scoreMotor.setControl(new DutyCycleOut(scorePower));
    }
}
