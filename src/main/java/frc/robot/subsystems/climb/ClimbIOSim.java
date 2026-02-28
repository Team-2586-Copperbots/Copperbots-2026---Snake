package frc.robot.subsystems.climb;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.subsystems.climb.Climb.ClimbPosition;

public class ClimbIOSim implements ClimbIO {
    private Double speed = 0.0;
    private ClimbPosition position = ClimbPosition.DOWN;
    private boolean positionVoltage = false;

    // private LinearSystemSim sim =

    public ClimbIOSim() {

    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.position = position;
        inputs.speed = speed;
        inputs.positionVoltage = positionVoltage;
    }

    @Override
    public void setSpeed(double speed) {
        this.speed = speed;
        this.positionVoltage = false;
    }
    @Override
    public void setPosition(ClimbPosition position) {
        this.position = position;
        this.positionVoltage = true;
    }
}
