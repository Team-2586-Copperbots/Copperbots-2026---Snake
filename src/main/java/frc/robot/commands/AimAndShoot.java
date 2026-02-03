package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AimAndShoot extends Command {
    private ShooterSubsystem shooterSubsystem;
    private TurretSubsystem turretSubsystem;
    private CommandSwerveDrivetrain drivetrainSubsystem;
    private Pose3d target;

    public AimAndShoot(ShooterSubsystem ShooterSubsystem, TurretSubsystem TurretSubsystem,
            CommandSwerveDrivetrain DrivetraineSubsystem, Pose3d Target) {
        this.shooterSubsystem = ShooterSubsystem;
        this.turretSubsystem = TurretSubsystem;
        this.drivetrainSubsystem = DrivetraineSubsystem;
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
        turretSubsystem.aimAtHub(drivetrainSubsystem);
        shooterSubsystem
                .setShooterSpeed(Utils.shooterSpeedFromDistance(target, Utils.pose3dForShooter(drivetrainSubsystem)));

    }

    @Override
    public boolean isFinished() {
        if (Math.abs(shooterSubsystem.getMotor1Speed()) < 0.5) {
            return true;
        }
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

}
