package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

public class AimAndShoot extends Command {
    private Shooter Shooter;
    private Turret Turret;
    private Drive Drive;
    private Pose2d target;

    public AimAndShoot(Shooter ShooterSubsystem, Turret TurretSubsystem,
            Drive Drivetrain) {
        this.Shooter = ShooterSubsystem;
        this.Turret = TurretSubsystem;
        this.Drive = Drivetrain;
        this.target = Utils.findTarget(Drivetrain);
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
        Turret.setTurretRotationTarget(Utils.getAngleToHubWithVelocity(Drive));
        Shooter.setShooterSpeedSet(
                Utils.shooterSpeedFromDistance(Utils.distanceFromPose(target, Drive)));

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Turret.setTurretRotationTarget(0);
        Shooter.setShooterSpeedSet(Constants.SHOOTER_CONSTANTS.SHOOTER_IDLE_SPEED);
    }

}
