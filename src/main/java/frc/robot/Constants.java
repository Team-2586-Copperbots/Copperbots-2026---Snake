// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import java.io.IOException;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.AllianceFlipUtil;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // drive

  public static class ROBOT_PROPERTIES {
    private static RobotConfig ROBOT_CONFIG;

    // what on earth does throws exception mean and what are exceptions and what do
    // they do?
    public static RobotConfig getROBOT_CONFIG() {
      if (ROBOT_CONFIG == null) {
        try {
          ROBOT_CONFIG = RobotConfig.fromGUISettings();
        } catch (IOException | org.json.simple.parser.ParseException e) {
          throw new RuntimeException("Failed to load RobotConfig", e);
        }
      }
      return ROBOT_CONFIG;
    }

    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(3, 4, Units.degreesToRadians(720),
        Units.degreesToRadians(360));
    public static final double slowdownSpeed = 0.5;
  }

  // robotcontainer

  public static class OPERATOR_CONSTANTS {
    // controlers
    public static final int DRIVER_CONTROLER_PORT = 0;
    public static final int OPERATOR_CONTROLER_PORT = 1;
    public static final int TEST_CONTROLER1_PORT = 2;
    public static final int SIM_CONTROLER_PORT = 5;

    // drive speed limiter
    public static final double MAX_SPEED_LIMITER = 1;
    public static final double SLOW_SPEED_LIMITER = 0.2;

    public static final double IDLE_SHOOTER_SPEED = 45; // RPS
    public static final double ROLLER_SPEED = 0.8; // percentage
  }

  // math

  public static class FIELD_CONSTANTS {
    public static final Distance FIELD_LENGTH = Distance.ofBaseUnits(Units.inchesToMeters(651.22), Meters);
    public static final Distance FIELD_WIDTH = Distance.ofBaseUnits(Units.inchesToMeters(317.69), Meters);

    public static Pose2d CENTER_OF_HUB = new Pose2d();
    public static Pose2d BOTTOM_FULE_STORAGE = new Pose2d();
    public static Pose2d TOP_FULE_STORAGE = new Pose2d();

    public static void updatePositions() {
      CENTER_OF_HUB = AllianceFlipUtil.apply(
          new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84), Rotation2d.kZero));
      BOTTOM_FULE_STORAGE = AllianceFlipUtil.apply(new Pose2d(2.75, 1.6, Rotation2d.kZero));
      TOP_FULE_STORAGE = AllianceFlipUtil.apply(
          new Pose2d(AllianceFlipUtil.applyX(BOTTOM_FULE_STORAGE.getX()),
              (FIELD_WIDTH.abs(Meters) - BOTTOM_FULE_STORAGE.getY()), Rotation2d.kZero));
    }

  }

  public static class DRIVEBASE_TARGET_POSES {
    public static final Pose2d TEST_POSE2D = AllianceFlipUtil.apply(new Pose2d(2, 2, Rotation2d.kZero));
  }

  // hardware

  public static class CANIds {
    public static final CANBus Canivore = new CANBus("Subsystems");
    // shooter motors
    public static final int SHOOTER_MOTOR_1 = 21;
    public static final int SHOOTER_MOTOR_2 = 22;
    // 4 bar intake motors
    public static final int INTAKE_WRIST_MOTOR = 26;
    public static final int INTAKE_CANCODER = 28;
    public static final int INTAKE_ROLLER_MOTOR = 27;
    // intake motor
    public static final int INDEXER_MOTOR = 24;
    public static final int TOWER_MOTOR = 25;
    // climb motor
    public static final int CLIMB_MOTOR_1 = 29;

    public static final int CLIMB_MOTOR_2 = 30;
    // turret motor
    public static final int TURRET_TURN_MOTOR = 23;
    // candle
    public static final int CANDLE = 31;
  }

  public static class DIO_IDS {
    public static final int TURRET_LIMIT_SWITCH = 9;
  }

  // subsystems + simpuation

  public static class SHOOTER_CONSTANTS {
    public static final Distance HEIGHT_OF_WHEEL_OFF_GROUND = Distance.ofBaseUnits(0.64135, Meters); // in meters
    public static final double SHOOTER_WHEELE_CIRCUMFERENCE = 2 * 2 * Math.PI;
    public static final Angle SHOOTER_HOOD_ANGLE = Angle.ofBaseUnits(22.165, Degrees);
  }

  public static class TURRET_CONSTANTS {
    public static final double MOTOR_TO_RING_RATIO = (66 / 12) * 3 * 3 * 1.105598958;// weried mystery number from the
                                                                                     // ring of oditys
    public static final double TURRET_RING_MINIMUM_TO_ROBOT_BACK_OFFSET = 0.044;
    public static final double ROTATION_RANGE_IN_ROT = 0.79;
    public static final Pose2d TURRET_OFFSET_FROM_ROBOT_CENTER = new Pose2d(Units.inchesToMeters(-7.5),
        Units.inchesToMeters(-8.5), null);
  }

  public static class INTAKE_CONSTANTS {
    public static final double rotorToIntake = (5 / 1) * (59 / 24) * (45 / 20);
  }

  public static enum CANDLE_STRIPS {
    // TODO: fix indexe (plural?) when strips are made, for eskey
    BUILT_IN(0, 7),
    FIRST(8, 8 + 21),
    SECOND(78, 147),
    THIRD(148, 217);

    public final int start;
    public final int end;

    private CANDLE_STRIPS(int start, int end) {
      this.start = start;
      this.end = end;
    }
  }

}
