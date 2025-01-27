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
import frc.robot.pathgen.PathGen;
import frc.robot.pathgen.fieldobjects.*;
import frc.robot.subsystems.generated.TunerConstants;
import frc.robot.util.FieldConstant;

public final class Main {
  private Main() {}


  public static void main(String... args) {

    /* 
    PathGen pg = PathGen.getInstance();

    //adds to handler upon construction; dw about it
    new PolygonFO(
      new Translation2d(0.89, 3.84), 
      new Translation2d(1.8, 0.6),
      new Translation2d(4.57, 2.69));

    //use once when all fieldobjects have been registered
    pg.update_grid_fo((float)Math.hypot(Units.inchesToMeters(30), Units.inchesToMeters(29)) / 2);

    //difference between translation and pose https://www.desmos.com/calculator/j1qqnsaixy

    //pose sets initial velocity direction
    Trajectory traj_rotation = pg.generateTrajectory(new Pose2d(0, 0, new Rotation2d(Degrees.of(180))),new Pose2d(5, 5, new Rotation2d(Degrees.of(180))), 
    new TrajectoryConfig(
      6.37 / 4, 
      12.6 / 4));

    //translation sets initial direction towards next pivot point
    Trajectory traj_translation = pg.generateTrajectory(new Translation2d(0, 0),new Translation2d(5, 5), 
    new TrajectoryConfig(
      6.37 / 4, 
      12.6 / 4));

    for (State s: traj_translation.getStates()) {
      Translation2d t = s.poseMeters.getTranslation();
      System.out.println("(" + t.getX() + ", " + t.getY() + ")");
    }
    */
    
    RobotBase.startRobot(Robot::new);
  }
}
