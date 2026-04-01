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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.ROBOT_PROPERTIES;

public class VisionConstants {
        // AprilTag layout
        public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
                        .loadField(AprilTagFields.k2026RebuiltWelded);

        // Camera names, must match names configured on coprocessor
        public static String backCamera = "backCamera";
        public static String sideCamera = "sideCamera";

        // new transforms for positions of angled cameras
        public static Transform3d robotToBackCamera = new Transform3d(
                        // translation
                        Inches.of(-ROBOT_PROPERTIES.lengthOffset /* to the back of the robot then forward/in: */ + 2.5),
                        Inches.of(ROBOT_PROPERTIES.widthOffset /* to the LEFT(+) of the robot then in/right: */ + -3.5),
                        Inches.of(ROBOT_PROPERTIES.floorOffset + 10.35),
                        // rotation
                        new Rotation3d(
                                        Degrees.of(0), // no roll
                                        Degrees.of(-20), // pitch up 20 degres
                                        Degrees.of(180) // rotate around z 180 degres
                        ));

        public static Transform3d robotToSideCamera = new Transform3d(
                        // translation
                        Inches.of(-ROBOT_PROPERTIES.lengthOffset /* to the back of the robot then forward: */ + 13.5),
                        Inches.of(-ROBOT_PROPERTIES.widthOffset /* to the RIGHT(-) of the robot then in: */ + 0.75),
                        Inches.of(ROBOT_PROPERTIES.floorOffset + 7.6 /* 1x1 to turret ring */ + 1.223 /*3D print z */),
                        // rotation
                        new Rotation3d(
                                        Degrees.of(0), // no roll
                                        Degrees.of(-30), // pitch up 30 degres
                                        Degrees.of(270) // rotate around z 180 degres
                        ));

        // Basic filtering thresholds
        public static double maxAmbiguity = 0.3;
        public static double maxZError = 0.75;
        public static double maxYAngle = 30;

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
