package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AimAndShoot extends Command {
    private ShooterSubsystem Shooter;
    private TurretSubsystem Turret;
    private CommandSwerveDrivetrain Drivetrain;
    private Pose2d target;

    public AimAndShoot(ShooterSubsystem ShooterSubsystem, TurretSubsystem TurretSubsystem,
            CommandSwerveDrivetrain DrivetrainSubsystem, Pose2d Target) {
        this.Shooter = ShooterSubsystem;
        this.Turret = TurretSubsystem;
        this.Drivetrain = DrivetrainSubsystem;
        this.target = Target;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ShooterSubsystem);
        addRequirements(TurretSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Turret.setTurretRotation(Utils.getAngleToHub(Drivetrain));
        Shooter.setShooterSpeedSet(
                Utils.shooterSpeedFromDistance(Utils.distanceFromPose(Constants.PLACES.CENTER_OF_HUB, Drivetrain)));

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Turret.setTurretRotation(0);
        Shooter.setShooterSpeedSet(Constants.SHOOTER_CONSTANTS.SHOOTER_IDLE_SPEED);
    }

}
