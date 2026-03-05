package frc.robot.subsystems.vision.old;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PhotonSubsystem extends SubsystemBase {

    private final PhotonCamera camera1 = new PhotonCamera("TempCamera");
    private Matrix<N3, N1> curentStdDevs;
    // private final PhotonCameraSim aprilTag1Sim = new PhotonCameraSim(aprilTag1);

    public final Transform3d camera1Pos = Vision.backCameraTranslation;

    // private VisionSystemSim visonSim = new VisionSystemSim("Vision Sim");

    public final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera1Pos);

    public PhotonSubsystem() {
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        // for simulations
        // if (RobotBase.isSimulation()) {
        // aprilTag1Sim.prop.setCalibError(0.08, .02);
        // aprilTag1Sim.prop.setFPS(30);
        // aprilTag1Sim.prop.setAvgLatencyMs(35);
        // aprilTag1Sim.prop.setLatencyStdDevMs(5);
        // }
    }

    public Optional<EstimatedRobotPose> getRobotPose() {
        Optional<EstimatedRobotPose> robotPose = Optional.empty();

        for (PhotonPipelineResult change : camera1.getAllUnreadResults()) {
            robotPose = poseEstimator.update(change);
        }

        return robotPose;
    }

    public Matrix<N3, N1>  getAmbiguity() {
        updateEstimationStdDevs(getRobotPose(), getRobotPose().get().targetsUsed);
        return curentStdDevs;
    }

    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curentStdDevs = Vision.singleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Vision.singleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            for (var tgt : targets) {
                var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curentStdDevs = Vision.singleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = Vision.multiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curentStdDevs = estStdDevs;
            }
        }
    }

    public double getCamera1Yaw() {
        double yaw = Double.MAX_VALUE;
        // Camera processed a new frame since last
        // Get the last one in the list.
        var result = camera1.getLatestResult();
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
        if (getRobotPose().isPresent()) {
            Logger.recordOutput("estimated pose", getRobotPose().get().estimatedPose);
        }
        // Logger.recordOutput("estimated pose", getRobotPose().get().estimatedPose);
    }

}