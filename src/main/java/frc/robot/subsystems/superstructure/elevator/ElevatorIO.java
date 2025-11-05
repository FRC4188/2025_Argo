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

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;

    public boolean leaderConnected = false;
    public double leaderAppliedVolts = 0.0;
    public double leaderCurrentAmps = 0.0;

    public boolean followConnected = false;
    public double followAppliedVolts = 0.0;
    public double followCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run the motors at the specified open loop value. */
  public default void setOpenLoop(double output) {}

  /** Reset Elevator Zero value */
  public default void setZero() {}

  /** Run the elevator to the specified rotations. */
  public default void setPosition(Rotation2d rotation) {}

  /** Update the PID values of the elevator */
  public default void updatePID(double kP, double kI, double kD, double kG) {}
}
