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

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class IntakeIOReal implements IntakeIO {

  // Hardware objects
  private final TalonFX intakeTalon;

  // TalonFX Configuration
  private final TalonFXConfiguration intakeConfig;

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final VelocityVoltage velocityVoltageRequest =
      new VelocityVoltage(0.0).withEnableFOC(true);

  // Torque-current control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Inputs from drive motor
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;

  // Connection debouncers
  private final Debouncer falconConnectedDebounce = new Debouncer(0.5);

  public IntakeIOReal() {
    intakeTalon = new TalonFX(Constants.Id.kIntake, Constants.robot.rio);

    // Configure elevator
    intakeConfig = Constants.IntakeConstants.kInitialConfigs;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.Slot0 = Constants.IntakeConstants.kMotorGains;
    intakeConfig.Feedback.SensorToMechanismRatio = Constants.IntakeConstants.kGearRatio;
    intakeConfig.MotorOutput.Inverted =
        Constants.IntakeConstants.kMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> intakeTalon.getConfigurator().apply(intakeConfig, 0.25));

    // Create status signals
    velocity = intakeTalon.getVelocity();
    appliedVolts = intakeTalon.getMotorVoltage();
    currentAmps = intakeTalon.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, appliedVolts, currentAmps);
    ParentDevice.optimizeBusUtilizationForAll(intakeTalon);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    // Refresh all signals
    var intakeStatus = BaseStatusSignal.refreshAll(velocity, appliedVolts, currentAmps);

    // Update inputs
    inputs.falconConnected = falconConnectedDebounce.calculate(intakeStatus.isOK());
    inputs.velocityRotPerSec = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void updatePID(double kP, double kI, double kD) {
    intakeConfig.Slot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD);
    tryUntilOk(5, () -> intakeTalon.getConfigurator().apply(intakeConfig, 0.25));
  }

  @Override
  public boolean isStalled() {
    return currentAmps.getValueAsDouble() > Constants.IntakeConstants.kStallCurrent
        && velocity.getValueAsDouble() < 0.01;
  }

  @Override
  public void setOpenLoop(double output) {
    intakeTalon.setControl(
        switch (Constants.IntakeConstants.motorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setVelocity(double rps) {
    intakeTalon.setControl(
        switch (Constants.IntakeConstants.motorClosedLoopOutput) {
          case Voltage -> velocityVoltageRequest.withVelocity(rps);
          case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(rps);
        });
  }
}
