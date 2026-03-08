package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class IntakePID extends Command {
    private Intake intake;
    private Intake.IntakePosition position = null;
    private Double wristSpeed = Double.NaN;
    private double rollerSpeed;

    public IntakePID(Intake intake, IntakePosition position, double rollerSpeed) {
        this.intake = intake;
        this.position = position;
        this.rollerSpeed = rollerSpeed;

        addRequirements(intake);
    }

    public IntakePID(Intake intake, double wristSpeed, double rollerSpeed) {
        this.intake = intake;
        this.wristSpeed = wristSpeed;
        this.rollerSpeed = rollerSpeed;

        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (position != null) {
            intake.setIntakePositionTarget(position);
        } else if (!wristSpeed.isNaN()) {
            intake.setWristSpeed(wristSpeed);
        }
        intake.setRollerSpeed(rollerSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (!wristSpeed.isNaN()) {
            intake.setWristSpeed(0);
        }
    }

}
