package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PhotonSubsystem extends SubsystemBase {

    PhotonCamera aprilTag1 = new PhotonCamera("TempCamera");

    public final Transform3d aprilTag1Pos = new Transform3d(0.32766, 0, 0.244475, new Rotation3d());


    public final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            aprilTag1Pos);

    public PhotonSubsystem() {
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    }

    public Optional<EstimatedRobotPose> getRobotPose() {
        Optional<EstimatedRobotPose> robotPose = Optional.empty();

        for (PhotonPipelineResult change : aprilTag1.getAllUnreadResults()) {
            robotPose = poseEstimator.update(change);
        }

        return robotPose;
    }


    public double getCamera1Yaw() {
        double yaw = Double.MAX_VALUE;
        // Camera processed a new frame since last
        // Get the last one in the list.
        var result = aprilTag1.getLatestResult();
        if (result.hasTargets()) {
            var target = result.getBestTarget();
            yaw = target.getYaw();
        } else {
            yaw = 0;
        }
        return yaw;
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("AprilTag_1 yaw ", getCamera1Yaw());
        // SmartDashboard.putNumber("AprilTag_3 yaw ", getCamera3Yaw());
        // getRobotPose().get().estimatedPose
    }

}