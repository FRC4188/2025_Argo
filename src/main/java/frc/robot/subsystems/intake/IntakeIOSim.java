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

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class IntakeIOSim implements IntakeIO {
  // TunerConstants doesn't support separate sim constants, so they are declared locally
  private static final double KP = 0.05;
  private static final double KD = 0.0;
  private static final double KS = 0.0;
  private static final double KV_ROT =
      0.91035; // Same units as TunerConstants: (volt * secs) / rotation
  private static final double KV = 1.0 / Units.rotationsToRadians(1.0 / KV_ROT);
  private static final DCMotor GEARBOX = DCMotor.getFalcon500Foc(1);

  private final DCMotorSim intakeSim;

  private boolean intakeClosedLoop = false;
  private PIDController intakeController = new PIDController(KP, 0, KD);
  private double intakeFFVolts = 0.0;
  private double intakeAppliedVolts = 0.0;

  public IntakeIOSim() {
    // Create drive and turn sim models
    intakeSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 1.0, 1.0), GEARBOX);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Run closed-loop control
    if (intakeClosedLoop) {
      intakeAppliedVolts =
          intakeFFVolts + intakeController.calculate(intakeSim.getAngularVelocityRadPerSec());
    } else {
      intakeController.reset();
    }

    // Update simulation state
    intakeSim.setInputVoltage(MathUtil.clamp(intakeAppliedVolts, -12.0, 12.0));
    intakeSim.update(0.02);

    // Update drive inputs
    inputs.falconConnected = true;
    inputs.velocityRotPerSec = intakeSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = intakeAppliedVolts;
    inputs.currentAmps = Math.abs(intakeSim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double output) {
    intakeClosedLoop = false;
    intakeAppliedVolts = output;
  }

  @Override
  public void setVelocity(double velocityRotPerSec) {
    intakeClosedLoop = true;
    intakeFFVolts =
        KS * Math.signum(2 * Math.PI * velocityRotPerSec) + KV * 2 * Math.PI * velocityRotPerSec;
    intakeController.setSetpoint(2 * Math.PI * velocityRotPerSec);
  }
}
