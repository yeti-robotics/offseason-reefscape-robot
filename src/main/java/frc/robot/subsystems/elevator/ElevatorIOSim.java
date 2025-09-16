package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.sim.PhysicsSim;

import static frc.robot.constants.Constants.motorCANBus;


public class ElevatorIOSim implements ElevatorIO{
    public final TalonFX primaryElevatorMotor;
    public final TalonFX secondaryElevatorMotor;
    public final CANrange canRangeElevator;

    public ElevatorIOSim() {
        primaryElevatorMotor = new TalonFX(ElevatorConfigTalonFXReal.primaryElevatorMotorID, motorCANBus);
        secondaryElevatorMotor = new TalonFX(ElevatorConfigTalonFXReal.secondaryElevatorMotorID, motorCANBus);
        canRangeElevator = new CANrange(ElevatorConfigTalonFXReal.canRangeID, motorCANBus);
        PhysicsSim.getInstance().addTalonFX(primaryElevatorMotor);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionRotation = primaryElevatorMotor.getPosition().getValueAsDouble();
        inputs.velocityRPM = secondaryElevatorMotor.getVelocity().getValueAsDouble();
        inputs.isAtBottom = canRangeElevator.getIsDetected().getValue();
    }

    @Override
    public void moveToPosition(double position) {
        primaryElevatorMotor.setControl(new MotionMagicTorqueCurrentFOC(position));
    }
}
