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

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final Alert falconDisconnectedAlert;

  public Intake(IntakeIO io) {
    this.io = io;

    falconDisconnectedAlert = new Alert("Disconnected falcon motor.", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Update alerts
    falconDisconnectedAlert.set(!inputs.falconConnected);
  }

  /** Runs the intake to a set rpm */
  public void setVelocityRPM(double rpm) {
    io.setVelocity(rpm * 60);
  }

  /** Runs the intake with the specified output. */
  public void runVolts(double output) {
    io.setOpenLoop(output);
  }

  /** Disables output to intake. */
  public void stop() {
    io.setOpenLoop(0.0);
  }

  /** Returns the current intake velocity in rpm. */
  public double getRPM() {
    return inputs.velocityRotPerSec / 60;
  }

  public void updatePID(double p, double i, double d) {
    io.updatePID(p, i, d);
  }
}
