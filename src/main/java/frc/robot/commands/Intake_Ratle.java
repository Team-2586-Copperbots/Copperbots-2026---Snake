package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.Constants.INTAKE_CONSTANTS;
import frc.robot.Constants.OPERATOR_CONSTANTS;
import frc.robot.subsystems.intake.Intake;

public class Intake_Ratle extends Command {
    // unfinished class to rattle the intake in and out for auto
    // TODO: change to current bassed, mabey even a new class for a time based an current bassed
    // TODO: why does time bassed stop working after a time
    // needs to be runn constantly for the math with the intake roller to work
    private Intake Intake;
    private double lastActionTime;
    private double timeWhenHitCurrentThreshold;
    private double maxCurrent = 30;

    public Intake_Ratle(Intake Intake, boolean WCurrent) {
        this.Intake = Intake;
        // make a implementation with current based along side time based

        lastActionTime = System.currentTimeMillis();
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Intake.setRollerSpeed(OPERATOR_CONSTANTS.ROLLER_SPEED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Logger.recordOutput("time betwen", Math.abs(lastActionTime - System.currentTimeMillis()));
        if (Math.abs(lastActionTime - System.currentTimeMillis()) > INTAKE_CONSTANTS.timeBetwenRattaling) {
            Intake.setIntakePositionTarget(IntakePosition.OUT);
            if (Intake.getIsAtTarget()) {
                lastActionTime = System.currentTimeMillis();
            }
        } else {
            Intake.setIntakePositionTarget(IntakePosition.JUGGLE);
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Intake.setIntakePositionTarget(IntakePosition.OUT);
    }

}
