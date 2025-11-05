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

package frc.robot.subsystems.lift;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ElevatorIOSim implements ElevatorIO {
  // TunerConstants doesn't support separate sim constants, so they are declared locally

  private static final double ELE_KP = 8.0;
  private static final double ELE_KD = 0.0;
  private static final DCMotor ELE_GEARBOX = DCMotor.getKrakenX60Foc(2);

  private final DCMotorSim eleSim;

  private boolean eleClosedLoop = false;
  private PIDController eleController = new PIDController(ELE_KP, 0, ELE_KD);
  private double eleAppliedVolts = 0.0;

  public ElevatorIOSim() {
    // Create drive and turn sim models

    eleSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ELE_GEARBOX, 0.004, Constants.EleConstants.kGearRatio),
            ELE_GEARBOX);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Run closed-loop control

    if (eleClosedLoop) {
      eleAppliedVolts = eleController.calculate(eleSim.getAngularPositionRad());
    } else {
      eleController.reset();
    }

    // Update simulation state
    eleSim.setInputVoltage(MathUtil.clamp(eleAppliedVolts, -12.0, 12.0));
    eleSim.update(0.02);

    // Update drive inputs

    // Update turn inputs
    inputs.leaderConnected = true;
    inputs.followConnected = true;
    inputs.positionRad = eleSim.getAngularPositionRad();
    inputs.velocityRadPerSec = eleSim.getAngularVelocityRadPerSec();
    inputs.leaderAppliedVolts = eleAppliedVolts;
    inputs.leaderCurrentAmps = Math.abs(eleSim.getCurrentDrawAmps());
    inputs.followAppliedVolts = eleAppliedVolts;
    inputs.followCurrentAmps = Math.abs(eleSim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double output) {
    eleClosedLoop = false;
    eleAppliedVolts = output;
  }

  @Override
  public void setPosition(Rotation2d rotation) {
    eleClosedLoop = true;
    eleController.setSetpoint(rotation.getRadians());
  }
}
