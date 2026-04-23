// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.FIELD_CONSTANTS;

import org.littletonrobotics.junction.Logger;

public final class AllianceFlipUtil {

  public static boolean shouldFlip() {
    boolean shouldFlip = DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    return shouldFlip;
  }

  public static double applyX(double x) {
    return shouldFlip() ? FIELD_CONSTANTS.FIELD_LENGTH.in(Meters) - x : x;
  }

  public static double applyY(double y) {
    return shouldFlip() ? FIELD_CONSTANTS.FIELD_WIDTH.in(Meters) - y : y;
  }

  public static Rotation2d applyR(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  //
  // for line symetry fields
  public static Translation2d applyOnlyX(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), translation.getY());
  }

  public static Pose2d applyOnlyX(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(applyOnlyX(pose.getTranslation()), applyR(pose.getRotation()))
        : pose;
  }

  //
  // for rotational symetry fields (like 2026)
  public static Translation2d apply(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(apply(pose.getTranslation()), applyR(pose.getRotation()))
        : pose;
  }

  //
  //
  public static Translation3d applyOnlyX(Translation3d translation) {
    return new Translation3d(
        applyX(translation.getX()), translation.getY(), translation.getZ());
  }

  public static Rotation3d apply(Rotation3d rotation) {
    return shouldFlip() ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
  }

  public static Pose3d applyOnlyX(Pose3d pose) {
    return new Pose3d(applyOnlyX(pose.getTranslation()), apply(pose.getRotation()));
  }

  //

  public static Translation3d apply(Translation3d translation) {
    return new Translation3d(
        applyX(translation.getX()), applyY(translation.getY()), translation.getZ());
  }

  public static Pose3d apply(Pose3d pose) {
    return new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()));
  }

}
