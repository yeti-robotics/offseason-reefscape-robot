package frc.robot.util.sim;

import org.wpilib.units.measure.Angle;

public interface SimulatableMechanism {
    Angle getCurrentPosition();

    Angle getTargetPosition();
}
