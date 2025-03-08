// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.commands.autos.pathgen.PathGen;
import frc.robot.commands.autos.pathgen.fieldobjects.*;
import frc.robot.subsystems.generated.TunerConstants;
import frc.robot.util.FieldConstant;

public final class Main {
  private Main() {}


  public static void main(String... args) {
    
    RobotBase.startRobot(Robot::new);
  }
}