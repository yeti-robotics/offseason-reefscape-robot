package frc.robot.subsystems.vision.apriltag;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

public record AprilTagPose(
        Pose2d estimatedRobotPose, int numTags, double timestamp, Matrix<N3, N1> standardDeviations) {
    public static final Matrix<N3, N1> DEFAULT_STD_DEVS = VecBuilder.fill(0.1, 0.1, Math.toRadians(10));

    public AprilTagPose(Pose2d estimatedRobotPose, int numTags, double timestamp) {
        this(estimatedRobotPose, numTags, timestamp, DEFAULT_STD_DEVS);
    }
}
