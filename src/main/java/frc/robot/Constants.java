package frc.robot;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
 
  public static class controller {
    public static final int PILOT = 0;
    public static final int COPILOT = 1;
    public static final double DEADBAND = 0.2;
  }

  public static class robot {
    public static final String rio = "rio";
    public static final String canivore = "canivore";
    public static final double loopPeriodSecs = 0.02;

    public static final Mode simMode = Mode.REPLAY;
    public static final Mode currMode = RobotBase.isReal()? Mode.REAL : simMode;

    public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(8); // 12.6

    public static final double A_LENGTH = Units.inchesToMeters(29); //inches
    public static final double A_WIDTH = Units.inchesToMeters(30); //inches
    public static final double A_CROSSLENGTH = Math.hypot(A_LENGTH, A_WIDTH);

    public static final double B_LENGTH = A_LENGTH + Units.inchesToMeters(3.5) * 2;
    public static final double B_WIDTH = A_WIDTH + Units.inchesToMeters(3.5) * 2;
    public static final double B_CROSSLENGTH = Math.hypot(B_LENGTH, B_WIDTH);

    public static  final PIDConstants DRIVE_PID = new PIDConstants(5.0, 0.0, 0.0);
    public static  final PIDConstants TURN_PID = new PIDConstants(5.0, 0.0, 0.0);

    public static final Matrix<N3, N1> STATE_STD_DEVS = VecBuilder.fill(0.05, 0.05, 0.001);
    public static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(0.020, 0.020, 0.264);

    public static final PIDController CORRECTION_PID = new PIDController(0.1, 0.0, 0.006);
  }

  public static enum Mode {
    REAL,
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  public class Id{
    //pigeon 0
    //DT ids are 1->12
    public static final int kElevatorLead = 13;
    public static final int kElevatorFollow = 14;
    public static final int kWristCANCoder = 15;
    public static final int kClimber = 16;
    public static final int kWrist = 17;   
    public static final int kIntake = 18;
  }

  public static class ElevatorConstants{    
    public static final double kGearRatio = 30.0;
    public static final double kPitchRadius = 0.04475 / 2; //sproket size
    public static final double kTolerance = 0.05;

    public static final boolean isPro = false;

    public static final double kConversion = kGearRatio / (3 * kPitchRadius);//should i kill myself

    public static final double kMax_Vel = 5;
    public static final double kMax_Accel = 7.5;
    public static final Constraints kConstraints = new Constraints(kMax_Vel, kMax_Accel);

    public static final double kP = 25;
    public static final double kI = 0.0;  
    public static final double kD = 0.05;
    public static final double kFF = 0.19;
    
    public static final ProfiledPIDController ElePID = new ProfiledPIDController(kP, kI, kD, kConstraints);

    private static final CurrentLimitsConfigs kCurrentLimitsConfigs = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(100)
      .withSupplyCurrentLimit(80)
      .withStatorCurrentLimitEnable(true);

    private static final FeedbackConfigs kFeedbackConfigs = new FeedbackConfigs()
      .withSensorToMechanismRatio(kConversion);
    
    private static final MotionMagicConfigs kMagicConfigs = new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(RotationsPerSecond.of(1))
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

    private static final Slot0Configs kSlot0Configs = new Slot0Configs()
      .withGravityType(GravityTypeValue.Elevator_Static)
      .withKP(kP)
      .withKD(kD);

    private static final OpenLoopRampsConfigs kOpenLoopRampsConfigs = 
      new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(0.5);
    private static final ClosedLoopRampsConfigs kClosedLoopRampsConfigs = 
      new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.5);

    public static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration()
      .withCurrentLimits(kCurrentLimitsConfigs)
      .withFeedback(kFeedbackConfigs)
      .withMotionMagic(kMagicConfigs)
      .withSlot0(kSlot0Configs)
      .withClosedLoopRamps(kClosedLoopRampsConfigs)
      .withOpenLoopRamps(kOpenLoopRampsConfigs)
      .withMotorOutput(
        new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
      );
  }

  public static class WristConstants {
    public static final double kTolerance = 0.2;
    public static final double kGearRatio = 25.0;//TODO: soon to change
    public static final int kCurrentLimit = 60; 

    public static final double kMax_Vel = Units.degreesToRadians(960.0);
    public static final double kMax_Accel = Units.degreesToRadians(650.0);
    public static final Constraints kConstraints = new Constraints(kMax_Vel, kMax_Accel);

    public static final double kP = 2;
    public static final double kI = 0.0;
    public static final double kD = 0.1;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kG = 0.8;

    public static final ProfiledPIDController WristPID = new ProfiledPIDController(kP, kI, kD, kConstraints);
    public static final ArmFeedforward WristFF = new ArmFeedforward(kS, kG, kV, kA);

    public static final SignalsConfig signal = new SignalsConfig()
      .absoluteEncoderPositionAlwaysOn(true)
      .primaryEncoderPositionAlwaysOn(true);

    private static final ClosedLoopConfig tune = new ClosedLoopConfig()
      .pid(2.0, 0.0, 0.1)
      .apply(new MAXMotionConfig()
        .maxVelocity(kMax_Vel)
        .maxAcceleration(kMax_Accel)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal));

    public static final SparkMaxConfig config = (SparkMaxConfig) new SparkMaxConfig()
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(kCurrentLimit)
      .closedLoopRampRate(0.01)
      .apply(signal)
      .apply(tune);
    
    
  }

  public static class IntakeConstants {
    public static final double kStallCurrent = 35;

    private static final CurrentLimitsConfigs kCurrentLimitsConfigs = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(80)
      .withSupplyCurrentLimit(60)
      .withStatorCurrentLimitEnable(true);

    public static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration()
      .withCurrentLimits(kCurrentLimitsConfigs)
      .withMotorOutput(
        new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
      );
  }

  public static class ClimberConstants {

    public static final double kGearRatio = 1.0;
    public static final double kTolerance = 0.15;
    public static final boolean isFOC = false;

    private static final FeedbackConfigs kFeedbackConfigs = new FeedbackConfigs()
    .withSensorToMechanismRatio(kGearRatio);

    private static final CurrentLimitsConfigs kCurrentLimitsConfigs = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(80)
      .withSupplyCurrentLimit(60)
      .withStatorCurrentLimitEnable(true);

    public static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration()
      .withCurrentLimits(kCurrentLimitsConfigs)
      .withFeedback(kFeedbackConfigs)
      .withMotorOutput(
        new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
      );
  }
}