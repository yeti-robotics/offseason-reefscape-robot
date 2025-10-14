package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public enum ElevatorPosition {
    BOTTOM(0.0),
    L1(0.2),
    L2(1.038916),
    L3(2.493066),
    L4(5.05),
    HP_INAKE(0.0),
    BARGE(0.0),
    HIGH_ALGAE(3.403564),
    LOW_ALGAE(1.707520),
    PROCESSOR(0.0);

    private final Angle height;

    ElevatorPosition(double height) {
        this.height = Units.Rotations.of(height);
    }

    public Angle getHeight() {
        return height;
    }
}
