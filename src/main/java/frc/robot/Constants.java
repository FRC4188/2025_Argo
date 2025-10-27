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

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final PIDTuning pid_mode = PIDTuning.NONE;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum PIDTuning {
    NONE,

    /** Tuning individual speed modules */
    DRIVE_MOD,

    /** Tuning individual turn modules */
    TURN_MOD,

    /** Tuning profiled angle controller */
    ANGLE_PROF
  }

  public static class controller {
    public static final int PILOT = 0;
    public static final int COPILOT = 1;
    public static final double DEADBAND = 0.1;
  }

  public static class robot {
    public static final String rio = "rio";
    public static final String canivore = "canivore";
    public static final double loopPeriodSecs = 0.02;

    // arbitrary number
    public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(8);

    // source: adkit
    public static final double ANGLE_MAX_VELOCITY = 8.0;
    public static final double ANGLE_MAX_ACCELERATION = 20.0;

    // TODO: tune eventually
    public static final PIDConstants DRIVE_PID = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(5.0, 0.0, 0.0);

    public static final double ANGLE_KP = 5.0;
    public static final double ANGLE_KD = 0.4;
    public static final double ANGLE_TOL = 0.02;

    public static final double A_LENGTH = Units.inchesToMeters(29); // inches
    public static final double A_WIDTH = Units.inchesToMeters(30); // inches
    public static final double A_CROSSLENGTH = Math.hypot(A_LENGTH, A_WIDTH);

    public static final double B_LENGTH = A_LENGTH + Units.inchesToMeters(2.5) * 2;
    public static final double B_WIDTH = A_WIDTH + Units.inchesToMeters(2.5) * 2;
    public static final double B_CROSSLENGTH = Math.hypot(B_LENGTH, B_WIDTH);

    // Add if problem present
    // public static final PIDController CORRECTION_PID = new PIDController(0.1, 0.0, 0.006);
  }
}
