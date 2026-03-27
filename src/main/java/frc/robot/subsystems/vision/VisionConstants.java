// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.Constants.ROBOT_PROPERTIES;

public class VisionConstants {
        // AprilTag layout
        public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
                        .loadField(AprilTagFields.k2026RebuiltAndymark);

        // Camera names, must match names configured on coprocessor
        public static String backCamera = "backCamera";
        public static String sideCamera = "sideCamera";

        // new transrofms for positions of angled camras
        public static Transform3d robotToBackCamera = new Transform3d(
                        // translation
                        Distance.ofBaseUnits(2.5 - ROBOT_PROPERTIES.lengthOffset, Inches),
                        Distance.ofBaseUnits(-3.5 + ROBOT_PROPERTIES.widthOffset, Inches),
                        Distance.ofBaseUnits(10.35 + ROBOT_PROPERTIES.floorOffset, Inches),
                        // rotation
                        new Rotation3d(
                                        Angle.ofBaseUnits(0, Degrees),
                                        Angle.ofBaseUnits(-30, Degrees),
                                        Angle.ofBaseUnits(180, Degrees)));
                                        

        public static Transform3d robotToSideCamera = new Transform3d(
                        // translation
                        Distance.ofBaseUnits(13.5 + -ROBOT_PROPERTIES.lengthOffset, Inches),
                        Distance.ofBaseUnits(0.75 + -ROBOT_PROPERTIES.widthOffset, Inches),
                        Distance.ofBaseUnits(9.042 + ROBOT_PROPERTIES.floorOffset, Inches),
                        // rotation
                        new Rotation3d(
                                        Angle.ofBaseUnits(0, Degrees),
                                        Angle.ofBaseUnits(-30, Degrees),
                                        Angle.ofBaseUnits(270, Degrees)));

        // Basic filtering thresholds
        public static double maxAmbiguity = 0.3;
        public static double maxZError = 100;

        // Standard deviation baselines, for 1 meter distance and 1 tag
        // (Adjusted automatically based on distance and # of tags)
        public static double linearStdDevBaseline = 0.02; // Meters
        public static double angularStdDevBaseline = 0.06; // Radians

        // Standard deviation multipliers for each camera
        // (Adjust to trust some cameras more than others)
        public static double[] cameraStdDevFactors = new double[] {
                        1.0, // Camera 0
                        1.0 // Camera 1
        };

        // Multipliers to apply for MegaTag 2 observations
        public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
        public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

}
