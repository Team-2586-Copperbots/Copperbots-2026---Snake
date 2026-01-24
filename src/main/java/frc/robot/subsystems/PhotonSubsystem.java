package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PhotonSubsystem extends SubsystemBase {

    PhotonCamera aprilTag1 = new PhotonCamera("TempCamera");
    // PhotonCamera aprilTag3 = new PhotonCamera("Apriltag_3");

    // location of camera on the robot
    // escanaba camera:
    // public final Transform3d aprilTag1Pos = new Transform3d(0.3302, 0, 0.209, new
    // Rotation3d());
    //

    // new lssu 1 camera
    public final Transform3d aprilTag1Pos = new Transform3d(0.32766, 0, 0.244475, new Rotation3d());
    // //public final Transform3d aprilTag1Pos = new Transform3d(0.254, 0.2921,
    // 0.2032, new Rotation3d(0,0,Math.toRadians(45)));
    // public final Transform3d aprilTag1Pos = new Transform3d(0.2667, -0.29845,
    // 0.2794, new Rotation3d(0,0,Math.toRadians(45)));
    // //public final Transform3d aprilTag3Pos = new Transform3d(0.254, -0.2921,
    // 0.2032, new Rotation3d(0,0,Math.toRadians(-45)));
    // public final Transform3d aprilTag3Pos = new Transform3d(0.2667, 0.29845,
    // 0.2794, new Rotation3d(0,0,Math.toRadians(-45)));

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

    /*
     * public double getY1TranslationFromTarget(){
     * double yTranslation = Double.MAX_VALUE;
     * var results = aprilTag1.getAllUnreadResults();
     * if (!results.isEmpty()) {
     * // Camera processed a new frame since last
     * // Get the last one in the list.
     * var result = results.get(results.size() - 1);
     * if (result.hasTargets()) {
     * var target = result.getBestTarget();
     * double range = PhotonUtils.calculateDistanceToTargetMeters(
     * CAMERA_HEIGHT_METERS,
     * TARGET_HEIGHT_METERS,
     * CAMERA_PITCH_RADIANS,
     * Units.degreesToRadians(result.getBestTarget().getPitch()));
     * Translation2d translation =
     * PhotonUtils.estimateCameraToTargetTranslation(range,
     * Rotation2d.fromDegrees(-target.getYaw()));
     * yTranslation = translation.getY();
     * }
     * }
     * return yTranslation;
     * }
     */

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

    /*
     * public double getY2TranslationFromTarget(){
     * double yTranslation = Double.MAX_VALUE;
     * var results = aprilTag3.getAllUnreadResults();
     * if (!results.isEmpty()) {
     * // Camera processed a new frame since last
     * // Get the last one in the list.
     * var result = results.get(results.size() - 1);
     * if (result.hasTargets()) {
     * var target = result.getBestTarget();
     * double range = PhotonUtils.calculateDistanceToTargetMeters(
     * CAMERA_HEIGHT_METERS,
     * TARGET_HEIGHT_METERS,
     * CAMERA_PITCH_RADIANS,
     * Units.degreesToRadians(result.getBestTarget().getPitch()));
     * Translation2d translation =
     * PhotonUtils.estimateCameraToTargetTranslation(range,
     * Rotation2d.fromDegrees(-target.getYaw()));
     * yTranslation = translation.getY();
     * }
     * }
     * return yTranslation;
     * }
     */
    // public double getCamera3Yaw() {
    // double yaw = Double.MAX_VALUE;
    // var result = aprilTag3.getLatestResult();
    // if (result.hasTargets()) {
    // var target = result.getBestTarget();
    // yaw = target.getYaw();
    // }
    // return yaw;
    // }

    // puts coral/Algae status on smartDashboard
    @Override
    public void periodic() {
        SmartDashboard.putNumber("AprilTag_1 yaw ", getCamera1Yaw());
        // SmartDashboard.putNumber("AprilTag_3 yaw ", getCamera3Yaw());
        // getRobotPose().get().estimatedPose
     }

}