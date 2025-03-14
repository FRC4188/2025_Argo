// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String frontLL = "front_limelight";
  public static String backLL = "back_limelight";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;
  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };
    
  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  public static int algaeDetect = 0; //placeholder
  public static int aprilTagDetect = 1; //placeholder 
  //TODO: WORK ON PIPELINES AND FIND THESE VALUES
  public static double left_region = -10.0; //left region of cam (left side reef)
  public static double right_region = 10.0; //right region of cam (right side reef)

  //TODO: WORK ON PIPELINES AND FIND REGION VALUES TAKING INTO CONSIDERATION OF CROSSHAIR OFFSET FROM CENTER SINCE LL IS NOT CENTERED

  public class AprilTagPose{
    public static Pose3d tag1 = 
        new Pose3d(
            Units.inchesToMeters(657.37), 
            Units.inchesToMeters(25.80), 
            Units.inchesToMeters(58.50),
        new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(126.0)));
     
    public static Pose3d tag2 = 
        new Pose3d(
            Units.inchesToMeters(657.37), 
            Units.inchesToMeters(291.20), 
            Units.inchesToMeters(58.50),
        new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(234.0)));

    public static Pose3d tag3 = 
        new Pose3d(
            Units.inchesToMeters(455.15), 
            Units.inchesToMeters(317.15), 
            Units.inchesToMeters(51.25),
        new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(30.0), 
            Units.degreesToRadians(0.0)));

    public static Pose3d tag4 = 
        new Pose3d(
            Units.inchesToMeters(365.20), 
            Units.inchesToMeters(241.64), 
            Units.inchesToMeters(73.54),
        new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(30.0), 
            Units.degreesToRadians(0.0)));

    public static Pose3d tag5 = 
        new Pose3d(
            Units.inchesToMeters(365.20), 
            Units.inchesToMeters(75.39), 
            Units.inchesToMeters(73.54),
        new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(30.0), 
            Units.degreesToRadians(0.0)));

    public static Pose3d tag6 = 
        new Pose3d(
            Units.inchesToMeters(530.49), 
            Units.inchesToMeters(130.17), 
            Units.inchesToMeters(12.13),
        new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(300.0)));

    public static Pose3d tag7 = 
        new Pose3d(
            Units.inchesToMeters(546.87), 
            Units.inchesToMeters(158.80), 
            Units.inchesToMeters(12.13),
         new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(0.0)));

    public static Pose3d tag8 = 
        new Pose3d(
            Units.inchesToMeters(530.49), 
            Units.inchesToMeters(186.83), 
            Units.inchesToMeters(12.13),
        new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(60.0), 
            Units.degreesToRadians(0.0)));

    public static Pose3d tag9 = 
        new Pose3d(
            Units.inchesToMeters(497.77), 
            Units.inchesToMeters(186.83), 
            Units.inchesToMeters(12.13),
        new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(120.0)));

    public static Pose3d tag10 = 
        new Pose3d(
            Units.inchesToMeters(481.39), 
            Units.inchesToMeters(158.50), 
            Units.inchesToMeters(12.13),
        new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(180.0)));

    public static Pose3d tag11 = 
        new Pose3d(
            Units.inchesToMeters(497.77), 
            Units.inchesToMeters(130.177), 
            Units.inchesToMeters(12.13),
        new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(240.0)));

    public static Pose3d tag12 = 
        new Pose3d(
            Units.inchesToMeters(33.51), 
            Units.inchesToMeters(25.80), 
            Units.inchesToMeters(58.50),
        new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(54.0)));

    public static Pose3d tag13 = 
        new Pose3d(
            Units.inchesToMeters(33.51), 
            Units.inchesToMeters(291.20), 
            Units.inchesToMeters(58.50),
        new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(306.0)));
        
    public static Pose3d tag14 = 
        new Pose3d(
            Units.inchesToMeters(325.68), 
            Units.inchesToMeters(241.64), 
            Units.inchesToMeters(73.54),
        new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(30.0), 
            Units.degreesToRadians(180.0)));

    public static Pose3d tag15 = 
        new Pose3d(
            Units.inchesToMeters(325.68), 
            Units.inchesToMeters(75.39), 
            Units.inchesToMeters(73.54),
        new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(30.0), 
            Units.degreesToRadians(180.0)));

    public static Pose3d tag16 = 
        new Pose3d(
            Units.inchesToMeters(235.73), 
            Units.inchesToMeters(-0.15), 
            Units.inchesToMeters(51.25),
        new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(90.0)));
            
    public static Pose3d tag17 = 
        new Pose3d(
            Units.inchesToMeters(160.39), 
            Units.inchesToMeters(130.17), 
            Units.inchesToMeters(12.13),
        new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(240.0)));

    public static Pose3d tag18 = 
        new Pose3d(
            Units.inchesToMeters(144.0), 
            Units.inchesToMeters(158.50), 
            Units.inchesToMeters(12.13),
        new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(180.0)));
            
    public static Pose3d tag19 = 
        new Pose3d(
            Units.inchesToMeters(160.39), 
            Units.inchesToMeters(186.83), 
            Units.inchesToMeters(12.13),
        new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(120.0)));
         
    public static Pose3d tag20 = 
        new Pose3d(
            Units.inchesToMeters(193.10), 
            Units.inchesToMeters(186.83), 
            Units.inchesToMeters(12.13),
        new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(60.0)));
         
    public static Pose3d tag21 = 
        new Pose3d(
            Units.inchesToMeters(209.49), 
            Units.inchesToMeters(185.50), 
            Units.inchesToMeters(12.13),
        new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(0.0)));
         
    public static Pose3d tag22 = 
        new Pose3d(
            Units.inchesToMeters(193.10), 
            Units.inchesToMeters(130.17), 
            Units.inchesToMeters(12.13),
        new Rotation3d(
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(0.0), 
            Units.degreesToRadians(300.0)));
     
  }
}