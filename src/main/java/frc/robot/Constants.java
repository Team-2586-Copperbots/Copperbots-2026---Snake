// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

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
  public static class OperatorConstants {
    public static final int DriverControllerPort = 0;
    public static final double MAX_SPEED_LIMITER = .7;
  }

  public static class places {
    public static final Pose2d CENTER_OF_HUB = new Pose2d(4.62, 4.04, null);
  }
  public static class CANIds {
    // FIXME: fix CAN ideas
    // shooter motors
    public static final int SHOOTER_MOTOR_1_ID = 7;
    public static final int SHOOTER_MOTOR_2_ID = 10;
    // 4 bar intake motors
    public static final int INTAKE_MOVEMENT_MOTOR_ID = 4;
    public static final int INTAKE_SPINNER_MOTOR_ID = 5;
    // intake motor
    public static final int INDEXER_MOTOR = 9;
    // turret motors
    public static final int TURRET_TURN_MOTOR = 1;
    public static final int TURRET_SPIN_MOTOR = 2;
    public static final int TURRET_CANCODER_ID = 3;
  }

  public static class ShooterConstants {
    public static final double SHOOTER_SPEED = 20.0; // RPM
  }
  public static class TurretConstants {
    public static double CANCODER_OFFSET = 0;
    public static void setCANcoderOffset(double offset) {
      CANCODER_OFFSET = offset;
    }
  }
}
