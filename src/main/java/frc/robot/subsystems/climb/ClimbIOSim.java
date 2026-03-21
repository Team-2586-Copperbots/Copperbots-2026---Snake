package frc.robot.subsystems.climb;

import frc.robot.subsystems.climb.Climb.ClimbPosition;

public class ClimbIOSim implements ClimbIO {
    private Double speed = 0.0;
    private ClimbPosition position = ClimbPosition.DOWN;
    private boolean positionVoltage = false;

    public ClimbIOSim() {

    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.targetPosition = position;
        inputs.speed = speed;
        inputs.isPositionVoltage = positionVoltage;
    }

    @Override
    public void setSpeed(double speed) {
        this.speed = speed;
        this.positionVoltage = false;
    }

    @Override
    public void setTargetPosition(ClimbPosition position) {
        this.position = position;
        this.positionVoltage = true;
    }
}
