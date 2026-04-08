package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.SHOOTER_CONSTANTS;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.CANIds.SHOOTER_MOTOR_1;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private static Shooter instance = null;
    private ShooterIO io;
    private double setPoint = 0;

    private final Mechanism mech;
    private final SysIdRoutine sysid;

    public static Shooter getInstance() {
        if (instance == null) {
            switch (Constants.currentMode) {
                case REAL:
                    instance = new Shooter(new ShooterIOReal());
                    break;
                case SIM:
                    instance = new Shooter(new ShooterIOSim());
                    break;

                default:
                    instance = new Shooter(new ShooterIO() {
                    });
                    break;
            }
        }
        return instance;
    }

    private Shooter(ShooterIO io) {
        this.io = io;
        mech = new Mechanism((e) -> io.runVoltage(e.in(Volts)), null, null, "shooter");
        SysIdRoutine.Config config = new Config(null, null, Seconds.of(15),
                (state) -> Logger.recordOutput("Shooter/sysidState", state.toString()));
        sysid = new SysIdRoutine(config, mech);
    }

    // TODO: run at some point
    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> io.runVoltage(0.0))
                .withTimeout(1.0)
                .andThen(sysid.quasistatic(direction));
    }

    // TODO: run at some point
    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> io.runVoltage(0.0)).withTimeout(1.0).andThen(sysid.dynamic(direction));
    }

    // negative to decrese
    public void setShooterSpeedAjust(double amount) {
        this.setPoint += amount;
        io.setMotorSetpoint(setPoint);
    }

    // sets the absolute speed
    public void setShooterSpeedSet(double setPoint) {
        this.setPoint = setPoint;
        io.setMotorSetpoint(setPoint);
    }

    public double getMotor1Speed() {
        return io.getMotorInputs(CANIds.SHOOTER_MOTOR_1).velocity;
    }

    @AutoLogOutput(key = "Shooter/getAtTarget")
    public boolean isAtTarget() {
        return Math.abs(getMotor1Speed() - io.getMotorInputs(SHOOTER_MOTOR_1).setpoint) < SHOOTER_CONSTANTS.TOLERENCE;
    }

    @Override
    public void periodic() {
        io.updateInputs(null);
    }
}
