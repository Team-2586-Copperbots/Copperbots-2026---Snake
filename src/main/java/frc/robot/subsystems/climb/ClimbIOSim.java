package frc.robot.subsystems.climb;

import frc.robot.subsystems.climb.Climb.ClimbPosition;

public class ClimbIOSim implements ClimbIO {
    @SuppressWarnings("unused")
    private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    @SuppressWarnings("unused")
    private Double targetSpeed = 0.0;
    @SuppressWarnings("unused")
    private ClimbPosition position = ClimbPosition.DOWN;
    @SuppressWarnings("unused")
    private boolean positionVoltage = false;

    public ClimbIOSim() {
    }

    @Override
    public void updateAndLogInputs() {

    }

    @Override
    public void setSpeed(double speed) {
        this.targetSpeed = speed;
        this.positionVoltage = false;
    }

    @Override
    public void setTargetPosition(ClimbPosition position) {
        this.position = position;
        this.positionVoltage = true;
    }
}
