package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class Intake_PID extends Command {
    // class to move the intake
    // constructurs to use IntakePosition and wrist speed
    // needs to be run periodicly to do the logic for the roller
    private Intake intake;
    private Intake.IntakePosition position = null;
    private Double wristSpeed = Double.NaN;
    private double rollerSpeed;

    public Intake_PID(Intake intake, IntakePosition position, double rollerSpeed) {
        this.intake = intake;
        this.position = position;
        this.rollerSpeed = rollerSpeed;

        addRequirements(intake);
    }

    public Intake_PID(Intake intake, double wristSpeed, double rollerSpeed) {
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
        intake.setRollerSpeed(rollerSpeed);
        if (position != null) {
            intake.setIntakePositionTarget(position);
        } else if (!wristSpeed.isNaN()) {
            intake.setWristSpeed(wristSpeed);
            System.out.println("position is null");
        } else {
            System.out.println("Wrist speed is Nan");
        }
    }

    @Override
    public boolean isFinished() {
        // if (intake.getIsAtTarget() && position != null) {
        // return true;
        // }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (!wristSpeed.isNaN()) {
            intake.setWristSpeed(0);
        }
        intake.setRollerSpeed(rollerSpeed);
    }

}
