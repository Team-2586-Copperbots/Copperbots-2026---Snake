// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private double MaxSpeed = OperatorConstants.MAX_SPEED_LIMITER * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
                                                                                                                      // desired
                                                                                                                      // top
  // speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final PhotonSubsystem vision = new PhotonSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final CommandPS4Controller driveController = new CommandPS4Controller(
      OperatorConstants.DRIVER_CONTROLER_PORT);
  private final CommandPS4Controller operatorController = new CommandPS4Controller(
      OperatorConstants.OPERATOR_CONTROLER_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(driveController.getLeftY() * MaxSpeed) // Drive forward with
            // negative Y
            // (forward)
            .withVelocityY(driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with
        // negative X (left)
        ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.

    operatorController.triangle().whileTrue(shooter.setShooterSpeed(0));

    operatorController.square().whileTrue(shooter.setShooterSpeed(43));

    operatorController.cross().whileTrue(shooter.setShooterSpeed(45));

    operatorController.circle().whileTrue(shooter.setShooterSpeed(40));

    operatorController.L1().whileTrue(turret.setIntakePosition(0.0));

    operatorController.L2().whileTrue(turret.setIntakePosition(0.75));

    operatorController.povUp().whileTrue(turret.aimAtHub(drivetrain));

    operatorController.options().whileTrue(turret.setTurnMotor(0.1));
    operatorController.share().whileTrue(turret.setTurnMotor(-0.1));

    // operatorController.povLeft().whileTrue(shooter.setShooterSpeedAmount(20));

    // operatorController.povUp().whileTrue(new SequentialCommandGroup(
    //     shooter.runOnce(() -> shooter.setDynamicSpeedAjust(1)),
    //     shooter.setShooterSpeedToSpeed()));

    // operatorController.povDown().whileTrue(new SequentialCommandGroup(
    //     shooter.runOnce(() -> shooter.setDynamicSpeedAjust(-1)),
    //     shooter.setShooterSpeedToSpeed()));

  }

  /**
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

}
