// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

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
  }

  public static class OPERATOR_CONSTANTS {
    public static final int DRIVER_CONTROLER_PORT = 0;
    public static final int OPERATOR_CONTROLER_PORT = 1;
    public static final int TEST_CONTROLER_PORT = 2;
    public static final double MAX_SPEED_LIMITER = .7;
  }

  public static class PLACES {
    public static final Pose3d CENTER_OF_HUB = new Pose3d(4.62, 4.04, 1.8288, null);
  }

  public static class DRIVEBASE_TARGET_POSES {
    public static final Pose2d TEST_POSE2D = new Pose2d(2, 2, new Rotation2d(Units.degreesToRadians(0)));
  }

  public static class CANIds {
    // shooter motors
    public static final int SHOOTER_MOTOR_1_ID = 21;
    public static final int SHOOTER_MOTOR_2_ID = 22;
    // 4 bar intake motors
    public static final int INTAKE_MOVEMENT_MOTOR_ID = 26;
    public static final int INTAKE_CANCODER = 28;
    public static final int INTAKE_SPINNER_MOTOR_ID = 27;
    // intake motor
    public static final int INDEXER_MOTOR = 24;
    public static final int TOWER_MOTOR = 25;
    // turret motor
    public static final int TURRET_TURN_MOTOR = 23;

    public static final int CANDLE_ID = 31;
  }

  public static class DIO_IDS {

    public static final int TURRET_LIMIT_SWITCH = 2;
  }

  public static class SHOOTER_CONSTANTS {
    public static final double SHOOTER_SPEED = 20.0; // RPM
    public static final double HEIGHT_OF_WHEEL_OFF_GROUND = 0.66; // in meters
  }

  public static class TURRET_CONSTANTS {
    public static final double MOTOR_TO_RING_RATIO = (66 / 12) * 3 * 3 * 1.105598958;// weried mystery number from the
                                                                                     // ring of oditys
    public static final double TURRET_ZERO_TO_ROBOT_ZERO_OFFSET = 0.044;
    public static final double ROTATION_RANGE_IN_ROT = 0.894;
    public static final Pose2d TURRET_OFFSET_FROM_ROBOT_CENTER = new Pose2d(Units.inchesToMeters(-7.375),
        Units.inchesToMeters(-7.375), null);
    public static final double TURRET_DISTANCE_FROM_ROBOT_CENTER = Units.inchesToMeters(Math.sqrt(
        (Math.pow(TURRET_OFFSET_FROM_ROBOT_CENTER.getX(), 2) + Math.pow(TURRET_OFFSET_FROM_ROBOT_CENTER.getY(), 2))));
  }

  public static enum IntakePosition {
    IN(0),
    OUT(40),
    HALFWAY(12);

    private final int value;

    private IntakePosition(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }
  }

  public static enum CANDLE_STRIPS {
    // TODO: fix indexe (plural?) when strips are made
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
