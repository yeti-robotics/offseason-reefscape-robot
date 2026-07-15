package frc.robot.util.sim.vision;

import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;
import org.wpilib.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.simulation.VisionSystemSim;

public class AprilTagSimulator {
    VisionSystemSim visionSim;
    List<AprilTagCamSim> aprilTagCamSims;

    public AprilTagSimulator() {
        visionSim = new VisionSystemSim("main");
        aprilTagCamSims = new ArrayList<>();

        visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField));
    }

    public void addCamera(AprilTagCamSim aprilTagCamSim) {
        visionSim.addCamera(aprilTagCamSim.getCameraSim(), aprilTagCamSim.getTransform());
        aprilTagCamSims.add(aprilTagCamSim);
    }

    public void update(Pose2d pose) {
        visionSim.update(pose);
    }

    public List<AprilTagCamSim> getAprilTagCamSims() {
        return aprilTagCamSims;
    }
}
