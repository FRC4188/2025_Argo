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

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
    ANGLE_PROF,

    /** Tuning profiled angle controller */
    ELEVATOR,

    /** Tuning profiled angle controller */
    WRIST,

    /** Tuning profiled angle controller */
    INTAKE,
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
    public static final double ANGLE_MAX_VELOCITY = 3.0 * Math.PI;
    public static final double ANGLE_MAX_ACCELERATION = 20.0;

    // TODO: tune eventually
    public static final PIDConstants DRIVE_PID = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(5.0, 0.0, 0.0);

    public static final double ANGLE_KP = 5.0;
    public static final double ANGLE_KD = 0.4;
    public static final double ANGLE_TOL = 0.05;

    public static final double A_LENGTH = Units.inchesToMeters(29); // inches
    public static final double A_WIDTH = Units.inchesToMeters(30); // inches
    public static final double A_CROSSLENGTH = Math.hypot(A_LENGTH, A_WIDTH);

    public static final double B_LENGTH = A_LENGTH + Units.inchesToMeters(2.5) * 2;
    public static final double B_WIDTH = A_WIDTH + Units.inchesToMeters(2.5) * 2;
    public static final double B_CROSSLENGTH = Math.hypot(B_LENGTH, B_WIDTH);

    // Add if problem present
    // public static final PIDController CORRECTION_PID = new PIDController(0.1, 0.0, 0.006);
  }

  public class Id {
    // pigeon 0
    // DT ids are 1->12
    public static final int kElevatorLead = 13;
    public static final int kElevatorFollow = 14;
    public static final int kWristCANCoder = 15;
    public static final int kClimber = 16;
    public static final int kWrist = 17;
    public static final int kIntake = 18;
  }

  public static class EleConstants {
    public static final TalonFXConfiguration kInitialConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(20)
                    .withStatorCurrentLimit(30)
                    .withStatorCurrentLimitEnable(true));

    public static final double LOWEST_H = Units.inchesToMeters(9.13250);
    public static final double RANGE = Units.inchesToMeters(72);

    public static final double kGearRatio = 30.0;
    public static final double kPitchRadius = 0.04475 / 2; // sproket size
    public static final double kConversion = (3 * kPitchRadius);
    public static final double kMaxTorqueCurrent = 80;
    public static final boolean motorInverted = false;

    public static final double kTolerance = 0.05;

    public static final Slot0Configs motorGains =
        new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKP(25.0)
            .withKG(0.2);

    public static final ClosedLoopOutputType motorClosedLoopOutput = ClosedLoopOutputType.Voltage;
  }

  public static class IntakeConstants {
    public static final double kGearRatio = 5;
    public static final boolean kMotorInverted = false;
    public static final double kMaxTorqueCurrent = 80;
    public static final double kMaxVel = 4;

    public static final TalonFXConfiguration kInitialConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(60)
                    .withStatorCurrentLimit(80)
                    .withStatorCurrentLimitEnable(true));

    public static final Slot0Configs kMotorGains = new Slot0Configs().withKP(0.0).withKD(0.0);

    public static final ClosedLoopOutputType motorClosedLoopOutput = ClosedLoopOutputType.Voltage;
  }

  public static class WristConstants {
    public static final double kTolerance = 0.2;
    public static final double kGearRatio = 25.0; // TODO: soon to change
    public static final int kCurrentLimit = 40;

    public static final double kMax_Vel = Units.degreesToRadians(720.0);
    public static final double kMax_Accel = Units.degreesToRadians(720.0);
    public static final Constraints kConstraints = new Constraints(kMax_Vel, kMax_Accel);

    public static final boolean kSparkInverted = true;
    public static final double turnMotorReduction = 9424.0 / 203.0;
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

    public static final boolean kEncoderInverted = true;
    public static final double kEncoderOffset = -0.16772;
    public static final double kEncoderPositionFactor =
        2 * Math.PI / kGearRatio; // Rotations -> Radians
    public static final double kEncoderVelocityFactor =
        (2 * Math.PI) / 60.0 / kGearRatio; // RPM -> Rad/Sec

    public static final double kP = 0.17;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kG = -0.6;
    public static final double kV = 0.0;
    public static final double simkP = 0.0;
    public static final double simkD = 0.0;
    public static final double kPIDMinInput = 0; // Radians
    public static final double kPIDMaxInput = 2 * Math.PI; // Radians
  }
}
