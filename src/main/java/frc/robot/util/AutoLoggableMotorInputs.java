package frc.robot.util;

import static edu.wpi.first.units.Units.Rotations;

import java.nio.ByteBuffer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.fasterxml.jackson.databind.util.Converter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.RobotState;

public class AutoLoggableMotorInputs implements StructSerializable {
    public double volts;
    public double amps;
    public double setpoint;
    public double position;
    public boolean isOk;
    public boolean isClosedLoop;

    public AutoLoggableMotorInputs(TalonFX motor) {
        update(motor);
    }

    public AutoLoggableMotorInputs() {
        this.volts = 0;
        this.amps = 0;
        this.setpoint = 0;
        this.position = 0;
        this.isOk = false;
        this.isClosedLoop = false;
    }

    public void update(TalonFX motor) {
        volts = motor.getMotorVoltage().getValueAsDouble();
        amps = motor.getStatorCurrent().getValueAsDouble();
        setpoint = motor.getClosedLoopReference().getValueAsDouble();
        position = motor.getPosition().getValue().in(Rotations);
        isOk = motor.isAlive();
        isClosedLoop = motor.getControlMode().getValue() != ControlModeValue.DutyCycleOut;
    }

    // The Struct definition
    public static final Struct<AutoLoggableMotorInputs> struct = new Struct<AutoLoggableMotorInputs>() {
        @Override
        public Class<AutoLoggableMotorInputs> getTypeClass() {
            return AutoLoggableMotorInputs.class;
        }

        @Override
        public String getTypeName() {
            return "AutoLoggableMotor";
        }

        @Override
        public String getSchema() {
            return "double volts;double amps;double setpoint;double position;boolean isOk;boolean isClosedLoop";
        }

        @Override
        public int getSize() {
            // double (8) + double (8) + boolean (1, packed as byte)
            return kSizeBool * 2 + kSizeDouble * 4;
        }

        @Override
        public void pack(ByteBuffer bb, AutoLoggableMotorInputs motor) {
            bb.putDouble(motor.volts);
            bb.putDouble(motor.amps);
            bb.putDouble(motor.setpoint);
            bb.putDouble(motor.position);
            // Convert boolean to byte
            bb.put((byte) (motor.isOk ? 1 : 0));
            bb.put((byte) (motor.isClosedLoop ? 1 : 0));
        }

        @Override
        public AutoLoggableMotorInputs unpack(ByteBuffer bb) {
            AutoLoggableMotorInputs motor = new AutoLoggableMotorInputs(null);
            motor.volts = bb.getDouble();
            motor.amps = bb.getDouble();
            motor.setpoint = bb.getDouble();
            motor.position = bb.getDouble();
            motor.isOk = bb.get() != 0;
            motor.isClosedLoop = bb.get() != 0;
            return motor;
        }
    };

}
