package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final PIDTuning pid_mode = PIDTuning.ELEVATOR;

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static enum PIDTuning {
    NONE,
    DRIVE_MOD,
    TURN_MOD,
    ANGLE_PROF,
    ELEVATOR,
    WRIST,
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

    public static final double ANGLE_MAX_VELOCITY = 3.0 * Math.PI;
    public static final double ANGLE_MAX_ACCELERATION = 40.0;

    public static final PIDConstants DRIVE_PID = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(5.0, 0.0, 0.4);
    public static final double ANGLE_TOL = 0.05;
    public static final double ANGLE_FF = 0.5;

    public static final double A_LENGTH = Units.inchesToMeters(29);
    public static final double A_WIDTH = Units.inchesToMeters(30);
    public static final double A_CROSSLENGTH = Math.hypot(A_LENGTH, A_WIDTH);

    public static final double B_LENGTH = A_LENGTH + Units.inchesToMeters(3) * 2;
    public static final double B_WIDTH = A_WIDTH + Units.inchesToMeters(3) * 2;
    public static final double B_CROSSLENGTH = Math.hypot(B_LENGTH, B_WIDTH);
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
                    .withSupplyCurrentLimit(60)
                    .withStatorCurrentLimit(80)
                    .withStatorCurrentLimitEnable(true));

    public static final double BASE_HEIGHT = Units.inchesToMeters(9.13250);
    public static final double RANGE = Units.inchesToMeters(72);

    public static final double kGearRatio = 30.0;
    public static final double kConversion = 3 * 0.04475 * 0.5;
    public static final boolean motorInverted = false;

    public static final double kTolerance = 0.05;

    public static final Slot0Configs motorGains =
        new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKP(25.0)
            .withKG(0.2);

    public static final ClosedLoopOutputType motorClosedLoopOutput =
        ClosedLoopOutputType.Voltage;
  }

  public static class IntakeConstants {
    public static final double kGearRatio = 5;
    public static final boolean kMotorInverted = false;
    public static final double kMaxVel = 1100;
    public static final double kStallCurrent = 50;

    public static final TalonFXConfiguration kInitialConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(60)
                    .withStatorCurrentLimit(80)
                    .withStatorCurrentLimitEnable(true));

    public static final Slot0Configs kMotorGains =
        new Slot0Configs().withKP(2.0).withKI(20.0).withKD(0.0);

    public static final ClosedLoopOutputType motorClosedLoopOutput = ClosedLoopOutputType.Voltage;
  }

  public static class WristConstants {
    public static final double kTolerance = 0.35;
    public static final double kGearRatio = 25.0;
    public static final int kCurrentLimit = 40;

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
    public static final double kG = 0.6;
    public static final double kV = 0.0;
    public static final double simkP = 0.0;
    public static final double simkD = 0.0;
    public static final double kPIDMinInput = 0; // Radians
    public static final double kPIDMaxInput = 2 * Math.PI; // Radians
  }
}
