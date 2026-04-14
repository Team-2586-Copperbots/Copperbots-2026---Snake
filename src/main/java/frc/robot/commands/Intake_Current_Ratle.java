package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.Constants.INTAKE_CONSTANTS;
import frc.robot.Constants.OPERATOR_CONSTANTS;
import frc.robot.subsystems.intake.Intake;

public class Intake_Current_Ratle extends Command {
    // unfinished class to rattle the intake in and out for auto
    // needs to be runn constantly for the math with the intake roller to work
    private Intake Intake;
    private double timeWhenHitCurrentHitThreshold;
    private boolean wasStalled = true;
    private boolean goingOut = false;

    public Intake_Current_Ratle(Intake Intake) {
        this.Intake = Intake;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        Intake.setRollerSpeed(OPERATOR_CONSTANTS.ROLLER_SPEED);

        if ((Intake.getWristStalled() == true) && (wasStalled == false)) {
            timeWhenHitCurrentHitThreshold = System.currentTimeMillis();
            wasStalled = true;
        }

        if (Intake.getWristStalled() == false) {
            timeWhenHitCurrentHitThreshold = System.currentTimeMillis();
            wasStalled = false;
        }

        Logger.recordOutput("Intake/going out", goingOut);
        
        if (goingOut == true) {
            if (Intake.getIsAtTarget()) {
                goingOut = false;
            }
        } else {
            if (System.currentTimeMillis() - timeWhenHitCurrentHitThreshold > INTAKE_CONSTANTS.timeBetwenCurrentRattle) {
                Intake.setIntakePositionTarget(IntakePosition.OUT);
                goingOut = true;
            } else {
                Intake.setIntakePositionTarget(IntakePosition.JUGGLE);
            }
        }

    }

    @Override
    public boolean isFinished() {
        if (Intake.getWristTarget() == IntakePosition.IN && Intake.getIsAtTarget()) {
            Logger.recordOutput("Intake/isFinished", "the code for the intake rattle finished");
            return true;
        }
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            Intake.setIntakePositionTarget(IntakePosition.OUT);
        } else {
            Intake.setIntakePositionTarget(IntakePosition.IN);
        }
    }

}
