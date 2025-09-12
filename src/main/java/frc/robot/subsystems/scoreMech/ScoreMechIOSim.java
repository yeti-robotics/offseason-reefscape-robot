package frc.robot.subsystems.scoreMech;

import static frc.robot.constants.Constants.motorCANBus;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.sim.PhysicsSim;

public class ScoreMechIOSim implements ScoreMechIO {
    private TalonFX scoreMotor;
    private CANrange innerCANrange;
    private CANrange outerCANrange;

    public ScoreMechIOSim() {
        scoreMotor = new TalonFX(ScoreConfigs.scoreMotorID, motorCANBus);
        innerCANrange = new CANrange(ScoreConfigs.innerCANrangeID, motorCANBus);
        outerCANrange = new CANrange(ScoreConfigs.outerCANrangeID, motorCANBus);
        PhysicsSim.getInstance().addTalonFX(scoreMotor);
    }

    public void updateInputs(ScoreMechIOInputs inputs) {
        inputs.scoreVelocity = scoreMotor.getVelocity().getValueAsDouble();
        inputs.innerSensorDetected = innerCANrange.getIsDetected().getValue();
        inputs.outerSensorDetected = outerCANrange.getIsDetected().getValue();
    }

    @Override
    public void setPower(double scorePower) {
        scoreMotor.set(scorePower);
    }
}
