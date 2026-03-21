package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OPERATOR_CONSTANTS;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.GeneralUtils;

public class Turret_AimAndShoot extends Command {
    private Shooter Shooter;
    private Turret Turret;
    private Drive Drive;
    private Pose2d target;

    public Turret_AimAndShoot(Shooter ShooterSubsystem, Turret TurretSubsystem,
            Drive Drivetrain) {
        this.Shooter = ShooterSubsystem;
        this.Turret = TurretSubsystem;
        this.Drive = Drivetrain;
        this.target = GeneralUtils.findTarget(Drive);
        // this.target = FIELD_CONSTANTS.CENTER_OF_HUB;
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
        target = GeneralUtils.findTarget(Drive);
        Turret.setTurretRotationTarget(GeneralUtils.getAngleToTarget(Drive, target));
        Shooter.setShooterSpeedSet(
                GeneralUtils.shooterSpeedFromDistance(GeneralUtils.distanceFromPose(target, Drive)));

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Shooter.setShooterSpeedSet(OPERATOR_CONSTANTS.IDLE_SHOOTER_SPEED);
    }

}
