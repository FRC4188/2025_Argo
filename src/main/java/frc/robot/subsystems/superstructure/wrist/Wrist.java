// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Wrist {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private LoggedNetworkNumber targetAngle = new LoggedNetworkNumber("Wrist/Target", 0.0);
  private LoggedNetworkNumber wristkP = new LoggedNetworkNumber("Wrist/kP", 0.0);
  private LoggedNetworkNumber wristkI = new LoggedNetworkNumber("Wrist/kI", 0.0);
  private LoggedNetworkNumber wristkD = new LoggedNetworkNumber("Wrist/kD", 0.0);
  private LoggedNetworkNumber wristkG = new LoggedNetworkNumber("Wrist/kG", 0.0);

  private final Alert wristDisconnectedAlert;

  public Wrist(WristIO io) {
    this.io = io;
    wristDisconnectedAlert = new Alert("Disconnected Wrist", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);

    // Update alerts
    wristDisconnectedAlert.set(!inputs.wristConnected);
  }

  /** Runs the wrist to a specified angle */
  public void setPosition(Rotation2d angle) {
    if (Constants.pid_mode == Constants.PIDTuning.WRIST) {
      angle = Rotation2d.fromRadians(targetAngle.get());
    }

    angle = Rotation2d.fromRadians(MathUtil.clamp(angle.getRadians(), 0, Math.PI / 2));

    Logger.recordOutput("Wrist/SetPoint", angle);
    io.setPosition(angle);
  }

  /** Runs the wrist with a specified output. */
  public void runVolts(double output) {
    output = MathUtil.clamp(output, -12.0, 12.0);
    io.setOpenLoop(output);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setOpenLoop(0.0);
  }

  public void updatePID() {
    io.updatePID(wristkP.get(), wristkI.get(), wristkD.get(), wristkG.get());
  }

  /** Returns the current angle of the wrist. */
  public Rotation2d getAngle() {
    return inputs.wristPosition;
  }
}
