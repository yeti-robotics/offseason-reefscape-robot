package frc.robot.subsystems.elevator;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.constants.Constants.motorCANBus;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.sim.PhysicsSim;

public class ElevatorIOTalonFX implements ElevatorIO {
    public final TalonFX primaryElevatorMotor;
    public final TalonFX secondaryElevatorMotor;
    public final CANrange canRangeElevator;

    private final MotionMagicTorqueCurrentFOC magicRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(0);

    public ElevatorIOTalonFX() {
        primaryElevatorMotor = new TalonFX(ElevatorConfigTalonFXReal.primaryElevatorMotorID, motorCANBus);
        secondaryElevatorMotor = new TalonFX(ElevatorConfigTalonFXReal.secondaryElevatorMotorID, motorCANBus);
        canRangeElevator = new CANrange(ElevatorConfigTalonFXReal.canRangeID, motorCANBus);
        PhysicsSim.getInstance().addTalonFX(primaryElevatorMotor);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionRotation = primaryElevatorMotor.getPosition().getValueAsDouble();
        inputs.velocityRPM = primaryElevatorMotor.getVelocity().getValueAsDouble();
        inputs.isAtBottom = canRangeElevator.getIsDetected().getValue();
    }

    @Override
    public void moveToPosition(double position) {
        primaryElevatorMotor.setControl(magicRequest);
    }

    public Command zeroPosition() {
        return runOnce(() -> primaryElevatorMotor.setPosition(0));
    }

    public boolean atSetPoint(double desiredPosition, double positionTolerance) {
        return Math.abs(primaryElevatorMotor.getPosition().getValueAsDouble() - desiredPosition) < positionTolerance;
    }

    public Command moveTo(ElevatorPosition position) {
        return run(() -> primaryElevatorMotor.setControl(magicRequest.withPosition(position.ordinal())))
                .until(() -> atSetPoint(position.ordinal(), 0));
    }
}
