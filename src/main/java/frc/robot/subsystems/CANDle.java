package frc.robot.subsystems;

import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANDLE_CONSTANTS;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.CANDLE_CONSTANTS.STRIPS;

public class CANDle extends SubsystemBase {
    private CANdle candle;

    public CANDle() {
        candle = new CANdle(CANIds.CANDLE_ID);

    }

    public Command setLEDSTate(STRIPS strip, LEDState state) {
        return runOnce(() -> candle.setControl(
                new SolidColor(strip.start, strip.end).withColor(new RGBWColor(state.r, state.g, state.b))));
    }

    public Command fire(STRIPS strip) {
        return runOnce(
                () -> candle.setControl(new FireAnimation(strip.start, strip.end).withBrightness(.5).withCooling(.3)));
    }

    public static enum LEDState {

        ORANGE(255, 128, 0),
        GREEN(0, 128, 0),
        PURPLE(128, 0, 128),
        YELLOW(255, 128, 0),
        RED(255, 0, 0),
        BLACK(0, 0, 0),
        WHITE(255, 255, 255),
        COPPER(184, 115, 51),
        PINK(255, 50, 193),

        BLUE(0, 0, 225);

        public final int r;
        public final int g;
        public final int b;

        private LEDState(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }
}
