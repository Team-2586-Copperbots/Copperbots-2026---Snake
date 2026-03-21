package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class Intake_PID extends Command {
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
        // intake.refreshPosition();

    }

    @Override
    public void execute() {
        if (position != null) {
            intake.setIntakePositionTarget(position);
        } else if (!wristSpeed.isNaN()) {
            intake.setWristSpeed(wristSpeed);
            System.out.println("position is null");
        } else {
            System.out.println("Wrist speed is Nan");
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
        Logger.recordOutput("intake interupted: ", 0);
        System.out.println("inatke reset at : " + DriverStation.getMatchTime());
    }

}
