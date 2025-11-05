// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/** Physics sim implementation of module IO. */
public class WristIOSim implements WristIO {
  private final DCMotorSim wristSim;

  private boolean wristClosedLoop = false;
  private PIDController wristController =
      new PIDController(Constants.WristConstants.simkP, 0, Constants.WristConstants.simkD);
  private double wristAppliedVolts = 0.0;

  public WristIOSim() {
    // Create drive and turn sim models

    wristSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                Constants.WristConstants.turnGearbox,
                0.004,
                Constants.WristConstants.turnMotorReduction),
            Constants.WristConstants.turnGearbox);

    // Enable wrapping for turn PID
    wristController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    // Run closed-loop control

    if (wristClosedLoop) {
      wristAppliedVolts = wristController.calculate(wristSim.getAngularPositionRad());
    } else {
      wristController.reset();
    }

    // Update simulation state
    wristSim.setInputVoltage(MathUtil.clamp(wristAppliedVolts, -12.0, 12.0));
    wristSim.update(0.02);

    // Update turn inputs
    inputs.wristConnected = true;
    inputs.wristPosition = new Rotation2d(wristSim.getAngularPositionRad());
    inputs.wristAbsolutePosition = new Rotation2d(wristSim.getAngularPositionRad());
    inputs.velocityRadPerSec = wristSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = wristAppliedVolts;
    inputs.currentAmps = Math.abs(wristSim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double output) {
    wristClosedLoop = false;
    wristAppliedVolts = output;
  }

  @Override
  public void setPosition(Rotation2d rotation) {
    wristClosedLoop = true;
    wristController.setSetpoint(rotation.getRadians());
  }
}
