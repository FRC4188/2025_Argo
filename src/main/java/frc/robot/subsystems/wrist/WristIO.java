// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean wristConnected = false;
    public boolean cancoderConnected = false;

    public Rotation2d wristAbsolutePosition = new Rotation2d();
    public Rotation2d wristPosition = new Rotation2d();

    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {}

  /** Run the turn motor at the specified open loop value. */
  public default void setOpenLoop(double output) {}

  /** Run the turn motor to the specified rotations */
  public default void setPosition(Rotation2d rotation) {}

  /** Update the PID values of the elevator */
  public default void updatePID(double kP, double kI, double kD, double kG) {}
}
