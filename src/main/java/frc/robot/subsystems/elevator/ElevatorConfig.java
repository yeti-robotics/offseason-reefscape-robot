package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import frc.robot.Robot;

class ElevatorConfig {
    static final int primaryElevatorMotorID = 11;
    static final int secondaryElevatorMotorID = 22;
    static final int canRangeID = 1;

    static final double gearRatio = 44.0 / 18.0;

    private static final Slot0Configs SLOT_0_CONFIGS = Robot.isReal()
            ? new Slot0Configs()
                    .withKP(800)
                    .withKI(0)
                    .withKD(16)
                    .withKG(32)
                    .withKV(0.75)
                    .withKA(0.25)
                    .withKS(2)
                    .withGravityType(GravityTypeValue.Elevator_Static)
            : new Slot0Configs();

    static final TalonFXConfiguration primaryTalonFXConfigs = new TalonFXConfiguration()
            .withSlot0(SLOT_0_CONFIGS)
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicAcceleration(30)
                    .withMotionMagicCruiseVelocity(15)
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

    static final CANrangeConfiguration canRangeElevatorConfigs = new CANrangeConfiguration()
            .withFovParams(new FovParamsConfigs()
                    .withFOVCenterX(0)
                    .withFOVCenterY(0)
                    .withFOVRangeX(27)
                    .withFOVRangeY(27))
            .withProximityParams(new ProximityParamsConfigs()
                    .withProximityHysteresis(0.01)
                    .withProximityThreshold(0.07)
                    .withMinSignalStrengthForValidMeasurement(2000))
            .withToFParams(
                    new ToFParamsConfigs().withUpdateFrequency(50).withUpdateMode(UpdateModeValue.ShortRange100Hz));

    static final double HEIGHT_TOLERANCE = 0;
    static final double ELEVATOR_VELOCITY_TOLERANCE = 0;
}
