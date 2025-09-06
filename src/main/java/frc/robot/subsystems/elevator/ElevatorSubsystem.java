package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import static frc.robot.constants.Constants.motorCANBus;
import static frc.robot.subsystems.elevator.ElevatorConfig.*;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX primaryElevatorMotor =
            new TalonFX(ElevatorConfig.primaryElevatorMotorID, motorCANBus);
    private final TalonFX secondaryElevatorMotor =
            new TalonFX(ElevatorConfig.secondaryElevatorMotorID, motorCANBus);
    private final CANrange canRangeElevator = new CANrange(ElevatorConfig.canRangeID, motorCANBus);


    private final MotionMagicTorqueCurrentFOC magicRequest =
            new MotionMagicTorqueCurrentFOC(0).withSlot(0);
    private final StatusSignal<Angle> elevatorPosition = primaryElevatorMotor.getPosition();

    public ElevatorSubsystem(){
        primaryElevatorMotor.getConfigurator().apply(primaryTalonFXConfigs);
        secondaryElevatorMotor.getConfigurator().apply(secondaryTalonFXConfigs);
        secondaryElevatorMotor.setControl(
                new Follower(ElevatorConfig.primaryElevatorMotorID, true));
}
    public Command zeroPosition() {
        return runOnce(() -> primaryElevatorMotor.setPosition(0));
    }

    public boolean atSetPoint(double desiredPosition, double positionTolerance) {
        return Math.abs(primaryElevatorMotor.getPosition().getValueAsDouble() - desiredPosition) < positionTolerance;
    }

    public Command moveTo(ElevatorPosition position){
        return run(() -> primaryElevatorMotor.setControl(magicRequest.withPosition(position.ordinal()))).until(() -> atSetPoint(position.ordinal(), 0));
    }
}
