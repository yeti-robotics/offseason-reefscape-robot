package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Robot;

class ElevatorConfig {
    static final int primaryElevatorMotorID = 14;
    static final int secondaryElevatorMotorID = 15;
    static final int canRangeID = 16;

    static final double gearRatio = 44.0 / 18.0;

    private static final Slot0Configs SLOT_0_CONFIGS = Robot.isSimulation()
            ? new Slot0Configs()
                    .withKP(128)
                    .withKI(0)
                    .withKD(12)
                    .withKG(0)
                    .withKV(0)
                    .withKA(1)
                    .withKS(1)
                    .withGravityType(GravityTypeValue.Elevator_Static)
            : new Slot0Configs();

    static final TalonFXConfiguration primaryTalonFXConfigs = new TalonFXConfiguration()
            .withSlot0(SLOT_0_CONFIGS)
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(1)
                    .withMotionMagicCruiseVelocity(2)
                    .withMotionMagicJerk(0))
            .withFeedback(new FeedbackConfigs().withRotorToSensorRatio(1).withSensorToMechanismRatio(gearRatio))
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake));
    static final TalonFXConfiguration secondaryTalonFXConfigs = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(new FeedbackConfigs().withRotorToSensorRatio(1).withSensorToMechanismRatio(gearRatio));

    static final CANrangeConfiguration canRangeElevatorConfigs = new CANrangeConfiguration();

    static final double HEIGHT_TOLERANCE = 0;
    static final double ELEVATOR_VELOCITY_TOLERANCE = 0;
}
