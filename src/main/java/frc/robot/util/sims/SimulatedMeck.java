package frc.robot.util.sims;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;

public class SimulatedMeck {
    
    private Angle position = Rotations.of(0);
    private AngularVelocity velocity = RotationsPerSecond.of(0);
    private AngularAcceleration acceleration = RotationsPerSecondPerSecond.of(0);
    private MomentOfInertia Moi;
    private DCMotor motor;

    public SimulatedMeck(DCMotor motor) {
        this.motor = motor;
    }

    public void Update(Time time) {
    
        velocity.plus(acceleration.times(time));
        position.plus(velocity.times(time));

    }

    public void setAcceleration() {

    }

    public void setCurrent() {

    }

}
