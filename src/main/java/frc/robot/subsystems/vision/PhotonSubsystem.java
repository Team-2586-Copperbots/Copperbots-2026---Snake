package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PhotonSubsystem extends SubsystemBase {

    private final PhotonCamera camera1 = new PhotonCamera("TempCamera");
    // private final PhotonCameraSim aprilTag1Sim = new PhotonCameraSim(aprilTag1);

    public final Transform3d camera1Pos = new Transform3d(-0.3302, -0.14, .46514, new Rotation3d(new Rotation2d(Math.PI)));

    //private VisionSystemSim visonSim = new VisionSystemSim("Vision Sim");
    
    public final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera1Pos);

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

        for (PhotonPipelineResult change : camera1.getAllUnreadResults()) {
            robotPose = poseEstimator.update(change);
        }

        return robotPose;
    }

    // public double getAmbiguity() {
    //     if (getRobotPose().isPresent()) {
    //         EstimatedRobotPose result
    //     }
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
    //     // Optional<EstimatedRobotPose> robotPose = Optional.empty();

    //     // for (PhotonPipelineResult change : camera1.getAllUnreadResults()) {
    //     //     robotPose = poseEstimator.update(change);
    //     // }

    //     // return robotPose;
    // }


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
        Logger.recordOutput("estimated pose", getRobotPose().get().estimatedPose);
    }

}