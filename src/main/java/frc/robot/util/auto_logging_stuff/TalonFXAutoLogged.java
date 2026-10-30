package frc.robot.util.auto_logging_stuff;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

public class TalonFXAutoLogged extends TalonFX {
    protected final TalonFXInputsAutoLogged inputs;

    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Temperature> deviceTemp;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;

    private final StatusSignal<ControlModeValue> controlMode;
    private final StatusSignal<Double> closedLoopReference;

    private final BaseStatusSignal[] allSignals;

    private double lastUpdateTimestamp = -1.0;
    private static final double MIN_UPDATE_INTERVAL_SECS = 0.005; // 5ms loop deduplication threshold

    @AutoLog
    public static class TalonFXInputs {
        // public String name = "";
        public boolean isOk = false;

        public double volts = 0.0;
        public double amps = 0.0;
        public double temp = 0.0;

        public double position = 0.0;
        public double velocity = 0.0;

        public boolean isClosedLoop = false;
        public double setpoint = 0.0;
    }

    public TalonFXAutoLogged(int id, CANBus canbus) {
        super(id, canbus);
        inputs = new TalonFXInputsAutoLogged();

        statorCurrent = this.getStatorCurrent();
        motorVoltage = this.getMotorVoltage();
        deviceTemp = this.getDeviceTemp();

        position = this.getPosition();
        velocity = this.getVelocity();

        controlMode = this.getControlMode();
        closedLoopReference = this.getClosedLoopReference();

        allSignals = new BaseStatusSignal[] {
            position,
            velocity,
            closedLoopReference,
            motorVoltage,
            statorCurrent,
            controlMode,
            deviceTemp
        };

        if (RobotBase.isReal()) {
            // Configure telemetry rates to prevent CAN bus saturation:
            // 50 Hz for feedback loops & odometry
            BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, closedLoopReference);
            // 25 Hz for monitor signals
            BaseStatusSignal.setUpdateFrequencyForAll(25.0, motorVoltage, statorCurrent, controlMode);
            // 4 Hz for slow temperature telemetry
            BaseStatusSignal.setUpdateFrequencyForAll(4.0, deviceTemp);

            // Mute all other unconfigured default status frames (limits, faults, etc.)
            ParentDevice.optimizeBusUtilizationForAll(this);
        }

        updateInputs(true);
    }

    public TalonFXAutoLogged(int id) {
        this(id, new CANBus());
    }

    public TalonFXAutoLogged(int id, String canbus) {
        this(id, new CANBus(canbus));
    }

    /**
     * Returns all cached status signals for batch refreshing with BaseStatusSignal.refreshAll.
     */
    public BaseStatusSignal[] getSignals() {
        return allSignals;
    }

    public void updateInputs() {
        updateInputs(false);
    }

    public void updateInputs(boolean forceRefresh) {
        double now = Timer.getFPGATimestamp();
        if (!forceRefresh && lastUpdateTimestamp > 0 && (now - lastUpdateTimestamp) < MIN_UPDATE_INTERVAL_SECS) {
            return;
        }
        lastUpdateTimestamp = now;

        inputs.isOk = BaseStatusSignal.refreshAll(allSignals).isOK() && this.isAlive();

        inputs.amps = statorCurrent.getValueAsDouble();
        inputs.volts = motorVoltage.getValueAsDouble();
        inputs.temp = deviceTemp.getValueAsDouble();

        inputs.position = position.getValueAsDouble();
        inputs.velocity = velocity.getValueAsDouble();

        inputs.isClosedLoop = controlMode.getValue() == ControlModeValue.PositionVoltage;
        inputs.setpoint = closedLoopReference.getValueAsDouble();
    }

    /**
     * Unpacks telemetry from signals that were already refreshed in a multi-device batch call.
     */
    public void updateInputsFromRefreshedSignals() {
        lastUpdateTimestamp = Timer.getFPGATimestamp();

        inputs.isOk = position.getStatus().isOK() && this.isAlive();

        inputs.amps = statorCurrent.getValueAsDouble();
        inputs.volts = motorVoltage.getValueAsDouble();
        inputs.temp = deviceTemp.getValueAsDouble();

        inputs.position = position.getValueAsDouble();
        inputs.velocity = velocity.getValueAsDouble();

        inputs.isClosedLoop = controlMode.getValue() == ControlModeValue.PositionVoltage;
        inputs.setpoint = closedLoopReference.getValueAsDouble();
    }

    public TalonFXInputsAutoLogged getInputs() {
        updateInputs();
        return inputs;
    }
}
