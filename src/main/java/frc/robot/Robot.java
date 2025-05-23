// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.LinkedList;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.FieldConstant;
import frc.robot.util.LoggedCommandScheduler;

import static frc.robot.util.FieldConstant.Reef.AlgaeSource.*;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {

    m_robotContainer = new RobotContainer();

    switch(Constants.robot.currMode){
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();// Start logging! No more data receivers, replay sources, or metadata values may be added.
    SignalLogger.start();
    LoggedCommandScheduler.init(CommandScheduler.getInstance());
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);

    CommandScheduler.getInstance().run();

    Threads.setCurrentThreadPriority(false, 10);

    LoggedCommandScheduler.periodic();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.resetSimulation();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    FieldConstant.Net.reloadNscores();



    asources = new LinkedList<Pose2d>(Arrays.asList(
      alliance_src, left_brg_src, left_src_src, right_brg_src, right_src_src, mid_brg_src
    ));
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.teleInit();

    //TODO: fix after further autonomization
    asources = new LinkedList<Pose2d>(Arrays.asList(
      alliance_src, left_brg_src, left_src_src, right_brg_src, right_src_src, mid_brg_src
    ));

    FieldConstant.Net.reloadNscores();

    SignalLogger.start();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    SignalLogger.stop();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  /** This function is called periodically whilst in simulation. */  
  @Override 
  public void simulationPeriodic() {
      SimulatedArena.getInstance().simulationPeriodic();
      m_robotContainer.displaySimFieldToAdvantageScope();
  }
  

}
