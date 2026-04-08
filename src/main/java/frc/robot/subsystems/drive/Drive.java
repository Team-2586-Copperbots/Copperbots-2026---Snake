// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.FIELD_CONSTANTS;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeneralUtils;
import frc.robot.util.driveUtils.MathedClimbUtils;

import java.util.Set;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private static Drive instance = null;
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS = Math.max(
      Math.max(
          Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
          Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
      Math.max(
          Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
          Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = Constants.ROBOT_PROPERTIES.getROBOT_CONFIG().massKG;
  // private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = Constants.ROBOT_PROPERTIES.getROBOT_CONFIG().moduleConfig.wheelCOF;
  @SuppressWarnings("unused")
  private static final RobotConfig PP_CONFIG = Constants.ROBOT_PROPERTIES.getROBOT_CONFIG();
  // new RobotConfig(
  // ROBOT_MASS_KG,
  // ROBOT_MOI,
  // new ModuleConfig(
  // TunerConstants.FrontLeft.WheelRadius,
  // TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
  // WHEEL_COF,
  // DCMotor.getKrakenX60Foc(1)
  // .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
  // TunerConstants.FrontLeft.SlipCurrent,
  // 1),
  // getModuleTranslations());
  public static SwerveDriveSimulation driveSimulation = null;
  private Field2d field = new Field2d();

  private static DriveTrainSimulationConfig mapleSimConfig = null;

  public static DriveTrainSimulationConfig getMapleSimConfig() {
    if (mapleSimConfig != null)
      return mapleSimConfig;

    return mapleSimConfig = DriveTrainSimulationConfig.Default()
        .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
        .withCustomModuleTranslations(getModuleTranslations())
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(new SwerveModuleSimulationConfig(
            DCMotor.getKrakenX60(1),
            DCMotor.getFalcon500(1),
            TunerConstants.FrontLeft.DriveMotorGearRatio,
            TunerConstants.FrontLeft.SteerMotorGearRatio,
            Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
            Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
            Inches.of(2),
            KilogramSquareMeters.of(0.05),
            WHEEL_COF));
  }

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  public FollowPath.Builder pathBuilder;
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
      AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation,
      lastModulePositions, Pose2d.kZero);

  private final Consumer<Pose2d> resetSimulationPoseCallBack;

  // MARK:- getInstance
  public static Drive getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
          instance = new Drive(
              new GyroIOPigeon2(),
              new ModuleIOTalonFX(TunerConstants.FrontLeft),
              new ModuleIOTalonFX(TunerConstants.FrontRight),
              new ModuleIOTalonFX(TunerConstants.BackLeft),
              new ModuleIOTalonFX(TunerConstants.BackRight),
              (robotPose) -> {
              });
          break;
        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          driveSimulation = new SwerveDriveSimulation(Drive.getMapleSimConfig(),
              new Pose2d(2, 2, Rotation2d.kZero));
          SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
          instance = new Drive(
              new GyroIOSim(driveSimulation.getGyroSimulation()),
              new ModuleIOSim(driveSimulation.getModules()[0]),
              new ModuleIOSim(driveSimulation.getModules()[1]),
              new ModuleIOSim(driveSimulation.getModules()[2]),
              new ModuleIOSim(driveSimulation.getModules()[3]),
              driveSimulation::setSimulationWorldPose);
          break;
        default:// Replayed robot, disable IO implementations
          instance = new Drive(
              new GyroIO() {
              },
              new ModuleIO() {
              },
              new ModuleIO() {
              },
              new ModuleIO() {
              },
              new ModuleIO() {
              },
              (robotPose) -> {
              });
          break;
      }
    }
    return instance;
  }

  private Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Consumer<Pose2d> resetSimulationPoseCallBack) {
    this.gyroIO = gyroIO;
    this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // // Configure AutoBuilder for PathPlanner
    // AutoBuilder.configure(
    // this::getPose,
    // this::resetOdometry,
    // this::getChassisSpeeds,
    // this::runVelocity,
    // new PPHolonomicDriveController(
    // new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
    // PP_CONFIG,
    // () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
    // this);
    // Pathfinding.setPathfinder(new LocalADStarAK());
    // PathPlannerLogging.setLogActivePathCallback(
    // (activePath) -> {
    // Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new
    // Pose2d[0]));
    // });
    // PathPlannerLogging.setLogTargetPoseCallback(
    // (targetPose) -> {
    // Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
    // });

    buildBline();

    // Configure SysId
    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, null, null, (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  // MARK:- periodic
  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    Logger.recordOutput("climb pose", MathedClimbUtils.centerOfClimbPose);
    Logger.recordOutput("auto flip",
        AllianceFlipUtil.applyY(getPose().getY()) > AllianceFlipUtil.applyY(FIELD_CONSTANTS.CENTER_OF_HUB.getY()));
    Logger.recordOutput("target for turret", GeneralUtils.findTarget());
    field.setRobotPose(getPose());
    SmartDashboard.putData("field", field);
    resetSimulationPoseCallBack.accept(getPose());
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] = new SwerveModulePosition(
            modulePositions[moduleIndex].distanceMeters
                - lastModulePositions[moduleIndex].distanceMeters,
            modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  //
  //
  //
  //
  //
  //
  //

  // mythings

  private void buildBline() {
    // configure BLine
    // BLine:
    // 1. Set global constraints (once, at robot init)
    // Path.DefaultGlobalConstraints = new
    // Path.DefaultGlobalConstraints(kNumConfigAttempts, kNumConfigAttempts,
    // kNumConfigAttempts, m_lastSimTime, kSimLoopPeriod, m_drivetrainId,
    // kNumConfigAttempts);

    // 2. Create a FollowPath builder
    // importent!:
    // https://www.chiefdelphi.com/t/introducing-bline-a-new-rapid-polyline-autonomous-path-planning-suite/509778/89
    // how to mirrior
    pathBuilder = new FollowPath.Builder(
        this,
        this::getPose,
        this::getChassisSpeeds,
        (speeds) -> this.runVelocity(speeds),
        new PIDController(BLine_Constants.tkP, BLine_Constants.tkI, BLine_Constants.tkD),
        new PIDController(BLine_Constants.rkP, BLine_Constants.rkI, BLine_Constants.rkD),
        new PIDController(BLine_Constants.CTkP, BLine_Constants.CTkI, BLine_Constants.CTkD))
        .withDefaultShouldFlip();

    FollowPath.setPoseLoggingConsumer(pair -> {
      Logger.recordOutput(pair.getFirst(), pair.getSecond());
    });

    FollowPath.setTranslationListLoggingConsumer(pair -> {
      Logger.recordOutput(pair.getFirst(), pair.getSecond());
    });
  }

  public Command commandFromPath(Path path) {
    return pathBuilder.build(path);
  }

  public Path changeConstrains(Path path, PathConstraints constraints) {
    path.setPathConstraints(constraints);
    return path;
  }

  public DeferredCommand deferedCommandToPose(Pose2d pose) {
    return new DeferredCommand(() -> this.commandFromPath(pathFromPose(pose)), Set.of(this));
  }

  public Path pathFromString(String name) {
    return new Path(name);
  }

  public Path pathFromPose(Pose2d pose) {
    return new Path(new Path.Waypoint(pose));
  }

  public Command autoPathFromString(String name) {
    if (AllianceFlipUtil.applyY(getPose().getY()) > AllianceFlipUtil.applyY(FIELD_CONSTANTS.CENTER_OF_HUB.getY())) {
      return pathFromStringFlipable(name, false);
    } else {
      return pathFromStringFlipable(name, true);
    }
  }

  public Command pathFromStringFlipable(String name, boolean mirror) {
    Path path = new Path(name);
    if (mirror) {
      path.mirror();
    }
    return pathBuilder.build(path);
  }

  public Path autoMirrorPath(Path path) {
    if (AllianceFlipUtil.applyY(getPose().getY()) < AllianceFlipUtil.applyY(FIELD_CONSTANTS.CENTER_OF_HUB.getY())) {
      path.mirror();
    }
    return path;
  }

  public Command pathFlipable(Path path, boolean mirror) {
    if (mirror) {
      path.mirror();
    }
    return pathBuilder.build(path);
  }

  public Path setHighTolerence(Pose2d pose) {
    Path path = new Path(new Path.Waypoint(pose));
    path.setPathConstraints(
        new PathConstraints().setEndRotationToleranceDeg(BLine_Constants.highTolerenceRot)
            .setEndTranslationToleranceMeters(BLine_Constants.highTolerenceTranlation));
    return path;
  }

  public Path pathFromPoseWithConstraints(Pose2d target, PathConstraints constraints) {
    Path path = new Path(new Path.Waypoint(target));
    path.setPathConstraints(constraints);
    return path;
  }

  public Command resetHearding() {
    return runOnce(() -> resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.kZero)));
  }

  public Command cRunVelocity(ChassisSpeeds speeds) {
    return runEnd(() -> runVelocity(speeds), () -> runVelocity(new ChassisSpeeds()));
  }

  //
  //
  //
  //
  //
  //
  //

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement.
   * The modules will
   * return to their normal orientations the next time a nonzero velocity is
   * requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /**
   * Returns the module states (turn angles and drive velocities) for all of the
   * modules.
   */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /**
   * Returns the module positions (turn angles and drive positions) for all of the
   * modules.
   */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /**
   * Returns the average velocity of the modules in rotations/sec (Phoenix native
   * units).
   */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Resets the current odometry pose. */
  public Command commandResetOdometry(Pose2d pose) {
    return runOnce(() -> poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose));
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
        new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
