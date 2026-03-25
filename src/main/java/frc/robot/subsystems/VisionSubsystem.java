package frc.robot.subsystems;

import java.util.Comparator;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera = new PhotonCamera("LL-Turret-Camera");
    VisionSystemSim visionSim = new VisionSystemSim("turret");
    AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    SimCameraProperties cameraProp = new SimCameraProperties();
    PhotonCameraSim cameraSim;

    public VisionSubsystem() {
        if (RobotBase.isSimulation()) {
            cameraProp.setCalibration(640, 400, Rotation2d.fromDegrees(99.4));
            cameraProp.setCalibError(0.88, 0.08);
            cameraProp.setFPS(120);
            cameraProp.setAvgLatencyMs(35);
            cameraProp.setLatencyStdDevMs(5);

            cameraSim = new PhotonCameraSim(camera, cameraProp);

            Translation3d robotToCameraTrl = new Translation3d(-0.15, 0.15, 0.6);
            Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-55), 0);
            Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

            visionSim.addCamera(cameraSim, robotToCamera);
        }
    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            visionSim.update(new Pose2d());
        }

        camera.getAllUnreadResults().forEach((result) -> {
            if (result.hasTargets()) {
                var target = result.getBestTarget();

                if (target.getFiducialId() > 0) {
                    // Your logic here
                }
            }
        });
    }

    public Optional<PhotonTrackedTarget> getClosestTag() {
        return camera.getLatestResult().hasTargets()
                ? camera.getLatestResult().getTargets().stream()
                        .filter(t -> t.getFiducialId() > 0)
                        .min(Comparator.comparingDouble(
                                t -> t.getBestCameraToTarget().getTranslation().getNorm()))
                : Optional.empty();
    }
}