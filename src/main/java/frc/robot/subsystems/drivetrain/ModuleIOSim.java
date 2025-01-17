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

package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.generated.TunerConstants;
import frc.robot.util.PhoenixUtil;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSim implements ModuleIO {
  // TunerConstants doesn't support separate sim constants, so they are declared locally
  private static final double DRIVE_KP = 0.05;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KS = 0.0;
  private static final double DRIVE_KV_ROT =  0.91035; // Same units as TunerConstants: (volt * secs) / rotation
  private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
  private static final double TURN_KP = 8.0;
  private static final double TURN_KD = 0.0;
  private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private final SwerveModuleSimulation sim;
  private final SimulatedMotorController.GenericMotorController drive;
  private final SimulatedMotorController.GenericMotorController turn;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
  private PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(SwerveModuleSimulation sim) {
    // Create drive and turn sim models
    this.sim = sim;
    this.drive = sim
      .useGenericMotorControllerForDrive()
      .withCurrentLimit(Amps.of(TunerConstants.FrontLeft.SlipCurrent));

    this.turn = sim
      .useGenericControllerForSteer()
      .withCurrentLimit(Amps.of(20));

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(sim.getDriveWheelFinalSpeed().in(RadiansPerSecond));
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(sim.getSteerAbsoluteFacing().getRadians());
    } else {
      turnController.reset();
    }

    //update sim state
    drive.requestVoltage(Volts.of(driveAppliedVolts));
    turn.requestVoltage(Volts.of(turnAppliedVolts));

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = sim.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec =
            sim.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps =
            Math.abs(sim.getDriveMotorStatorCurrent().in(Amps));
    // Update turn inputs
    inputs.turnConnected = true;
      inputs.turnEncoderConnected = true;
      inputs.turnAbsolutePosition = sim.getSteerAbsoluteFacing();
      inputs.turnPosition = sim.getSteerAbsoluteFacing();
      inputs.turnVelocityRadPerSec =
              sim.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
      inputs.turnAppliedVolts = turnAppliedVolts;
      inputs.turnCurrentAmps =
              Math.abs(sim.getSteerMotorStatorCurrent().in(Amps));
        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
      inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
      inputs.odometryDrivePositionsRad = Arrays.stream(sim.getCachedDriveWheelFinalPositions())
              .mapToDouble(angle -> angle.in(Radians))
              .toArray();
      inputs.odometryTurnPositions = sim.getCachedSteerAbsolutePositions();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}