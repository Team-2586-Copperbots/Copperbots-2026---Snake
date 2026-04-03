package frc.robot.subsystems;

import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import frc.robot.subsystems.turret.Turret;
import frc.robot.Constants;
import frc.robot.Constants.LED_Strip;

import static edu.wpi.first.units.Units.Hertz;
import static frc.robot.Constants.CANIds.Canivore;

import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {
    private static LED instance = null;
    private CANdle candle;

    public static LED getInstance() {
        if (instance == null) {
            switch (Constants.currentMode) {
                case REAL:
                    instance = new LED();
                    break;
                // case SIM:
                // instance = null;
                // break;
                case REPLAY:
                    instance = new LED();
                    break;
                default:
                    instance = new LED();
                    break;
            }
        }
        return instance;
    }

    @Override
    public void periodic() {
        // setAutoState();
        // Logger.recordOutput("CANdle control", candle.getAppliedControl().toString());
    }

    private void setAutoState() {
        if (Turret.getInstance().canGetToTarget()) {
            setColor(LED_Strip.SECOND, LED_Colour.GREEN);
        } else {
            setColor(LED_Strip.SECOND, LED_Colour.RED);
        }
    }

    private LED() {
        candle = new CANdle(CANIds.CANDLE, Canivore);
    }

    public Command setColor(LED_Strip strip, LED_Colour colour) {
        return runOnce(() -> candle.setControl(
                new SolidColor(strip.start, strip.end).withColor(colour.getColor())));
    }

    public Command fire(LED_Strip strip) {
        return runOnce(
                () -> candle.setControl(new FireAnimation(strip.start, strip.end).withBrightness(.5).withCooling(.3)));
    }

    public Command boom(LED_Strip strip) {
        return runOnce(() -> candle.setControl(
                new TwinkleAnimation(strip.start, strip.end).withColor(LED_Colour.PURPLE.getColor())
                        .withFrameRate(Hertz.of(1)).withMaxLEDsOnProportion(1)));
    }

    public static enum LED_Colour {
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

        private LED_Colour(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }

        public RGBWColor getColor() {
            return new RGBWColor(r, g, b);
        }
    }
}
