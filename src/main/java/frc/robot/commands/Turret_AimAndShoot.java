package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OPERATOR_CONSTANTS;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.GeneralUtils;

public class Turret_AimAndShoot extends Command {
    // class to Aim the turret AND bring the shooter up to speed
    // could proably be done using a similar structure to
    // Climb_AutoClimb_Sequence.java with a static get command that callse auto aim
    // and auto shoot
    private Shooter Shooter;
    private Turret Turret;

    public Turret_AimAndShoot(Shooter ShooterSubsystem, Turret TurretSubsystem) {
        this.Shooter = ShooterSubsystem;
        this.Turret = TurretSubsystem;

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
        Turret.setTurretRotationTarget(GeneralUtils.getAngleToTarget());
        Shooter.setShooterSpeedSet(GeneralUtils.shooterSpeedFromTarget());

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Shooter.setShooterSpeedSet(OPERATOR_CONSTANTS.IDLE_SHOOTER_SPEED);
        Turret.setTurretRotationTarget(0);
    }

}
