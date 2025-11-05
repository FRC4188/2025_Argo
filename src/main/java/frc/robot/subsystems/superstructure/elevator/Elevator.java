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

package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Elevator {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Alert leaderDisconnectedAlert;
  private final Alert followerDisconnectedAlert;

  private LoggedNetworkNumber targetHeight = new LoggedNetworkNumber("Elevator/Target", 0.0);
  private LoggedNetworkNumber elekP = new LoggedNetworkNumber("Elevator/kP", 0.0);
  private LoggedNetworkNumber elekI = new LoggedNetworkNumber("Elevator/kI", 0.0);
  private LoggedNetworkNumber elekD = new LoggedNetworkNumber("Elevator/kD", 0.0);
  private LoggedNetworkNumber elekG = new LoggedNetworkNumber("Elevator/kG", 0.0);

  public Elevator(ElevatorIO io) {
    this.io = io;

    leaderDisconnectedAlert = new Alert("Disconnected leader motor.", AlertType.kError);
    followerDisconnectedAlert = new Alert("Disconnected follower motor.", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Update alerts
    leaderDisconnectedAlert.set(!inputs.leaderConnected);
    followerDisconnectedAlert.set(!inputs.followConnected);
  }

  /** Runs the elevator to a set height meters */
  public void setHeight(double height) {
    if (Constants.pid_mode == Constants.PIDTuning.ELEVATOR) {
      height = targetHeight.get();
    }

    Logger.recordOutput("Elevator/SetPoint", height);
    height = MathUtil.clamp(height, 0, Constants.EleConstants.RANGE);

    io.setPosition(new Rotation2d(height / Constants.EleConstants.kConversion));
  }

  /** Runs the elevator with the specified output. */
  public void runVolts(double output) {
    output = MathUtil.clamp(output, -12.0, 12.0);
    io.setOpenLoop(output);
  }

  public void setZero() {
    io.setZero();
  }

  /** Disables output to elevator. */
  public void stop() {
    io.setOpenLoop(0.0);
  }

  @AutoLogOutput(key = "Elevator/Height Meters")
  /** Returns the current elevator position in meters. */
  public double getPositionMeters() {
    return inputs.positionRad * Constants.EleConstants.kConversion;
  }

  /** Returns the current elevator velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.velocityRadPerSec * Constants.EleConstants.kConversion;
  }

  public void updatePID() {
    io.updatePID(elekP.get(), elekI.get(), elekD.get(), elekG.get());
  }
}
