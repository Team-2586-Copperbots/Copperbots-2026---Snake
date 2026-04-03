package frc.robot.util.auto_loggint_stuff;

import java.nio.ByteBuffer;

import edu.wpi.first.util.struct.Struct;

public class AutoLoggableMotorStruct implements Struct<AutoLoggableMotor> {
    @Override
    public Class<AutoLoggableMotor> getTypeClass() {
        return AutoLoggableMotor.class;
    }

    @Override
    public String getTypeName() {
        return "AutoLoggableMotor";
    }

    @Override
    public int getSize() {
        // double (8) + double (8) + boolean (1, packed as byte)
        return kSizeBool * 2 + kSizeDouble * 4;
    }

    @Override
    public void pack(ByteBuffer bb, AutoLoggableMotor motor) {
        bb.putDouble(motor.volts);
        bb.putDouble(motor.amps);
        bb.putDouble(motor.setpoint);
        bb.putDouble(motor.position);
        // Convert boolean to byte
        bb.put((byte) (motor.isOk ? 1 : 0));
        bb.put((byte) (motor.isClosedLoop ? 1 : 0));
    }

    @Override
    public AutoLoggableMotor unpack(ByteBuffer bb) {
        AutoLoggableMotor motor = new AutoLoggableMotor(null);
        motor.volts = bb.getDouble();
        motor.amps = bb.getDouble();
        motor.setpoint = bb.getDouble();
        motor.position = bb.getDouble();
        motor.isOk = bb.get() != 0;
        motor.isClosedLoop = bb.get() != 0;
        return motor;
    }

    @Override
    public String getSchema() {
        return "double volts;double amps;double setpoint;double position;boolean isOk;boolean isClosedLoop";
    }

}
