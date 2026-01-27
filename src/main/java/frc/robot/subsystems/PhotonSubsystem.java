package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PhotonSubsystem extends SubsystemBase {

    private final PhotonCamera aprilTag1 = new PhotonCamera("TempCamera");
    // private final PhotonCameraSim aprilTag1Sim = new PhotonCameraSim(aprilTag1);

    public final Transform3d aprilTag1Pos = new Transform3d(0.3302, 0, 0.2714625, new Rotation3d());

    //private VisionSystemSim visonSim = new VisionSystemSim("Vision Sim");
    
    public final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            aprilTag1Pos);

    public PhotonSubsystem() {
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        // for simulations
        // if (RobotBase.isSimulation()) {
        //     aprilTag1Sim.prop.setCalibError(0.08, .02);
        //     aprilTag1Sim.prop.setFPS(30);
        //     aprilTag1Sim.prop.setAvgLatencyMs(35);
        //     aprilTag1Sim.prop.setLatencyStdDevMs(5);
        // }
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