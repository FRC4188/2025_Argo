package frc.robot;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
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
    public static final double TRIGGER_THRESHOLD = 0.6;
  }

  public static class robot {

    public static final String rio = "rio";
    public static final String canivore = "canivore";
    public static final double loopPeriodSecs = 0.02;

    public static final Mode currMode = RobotBase.isReal()? Mode.REAL : Mode.SIM;

    public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(8); // 12.6

    public static final double A_LENGTH = Units.inchesToMeters(30); //inches
    public static final double A_WIDTH = Units.inchesToMeters(29); //inches
    public static final double A_CROSSLENGTH = Math.hypot(A_LENGTH, A_WIDTH);

    public static final double B_LENGTH = A_LENGTH + Units.inchesToMeters(3.2) * 2;
    public static final double B_WIDTH = Units.inchesToMeters(3.2) * 2;
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
    public static final int kWrist = 17;   
    public static final int kIntake = 18;
  }

  public static class ElevatorConstants{    
    public static final double kGearRatio = 30.0;
    public static final double kPitchRadius = 0.04475 / 2; //TODO: fix this
    public static final double kTolerance = 0.05;

    public static final boolean isPro = false;

    public static final double kConversion = 3 * 2 * Math.PI * kPitchRadius; 

    public static final double kMax_Vel = 3;
    public static final double kMax_Accel = 2;
    public static final Constraints kConstraints = new Constraints(kMax_Vel, kMax_Accel);

    public static final double kP = 25;
    public static final double kI = 0.0;  
    public static final double kD = 0.05;
    public static final double kFF = 0.19;
    
    public static final ProfiledPIDController ElePID = new ProfiledPIDController(kP, kI, kD, kConstraints);

    public static final ProfiledPIDController SimElePID = new ProfiledPIDController(
      10, 0, 0, new Constraints(20, 20)
      );

    private static final CurrentLimitsConfigs kCurrentLimitsConfigs = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(80)
      .withSupplyCurrentLimit(60)
      .withStatorCurrentLimitEnable(true);

    private static final FeedbackConfigs kFeedbackConfigs = new FeedbackConfigs()
      .withSensorToMechanismRatio(kGearRatio);
    
    private static final MotionMagicConfigs kMagicConfigs = new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(RotationsPerSecond.of(1))
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

    private static final Slot0Configs kSlot0Configs = new Slot0Configs()
      .withGravityType(GravityTypeValue.Elevator_Static)
      .withKP(kP)
      .withKD(kD);

    private static final OpenLoopRampsConfigs kOpenLoopRampsConfigs = new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(0.5);
    private static final ClosedLoopRampsConfigs kClosedLoopRampsConfigs = new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.5);

    public static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration()
      .withCurrentLimits(kCurrentLimitsConfigs)
      .withFeedback(kFeedbackConfigs)
      .withMotionMagic(kMagicConfigs)
      .withSlot0(kSlot0Configs)
      .withClosedLoopRamps(kClosedLoopRampsConfigs)
      .withOpenLoopRamps(kOpenLoopRampsConfigs);
  }

  public static class WristConstants {
    public static final double koffsetFromCenter = Units.inchesToMeters(2.668); //inches
    public static final double kTolerance = 0.1;
    public static final double kGearRatio = 1.0 / 25.0;
    public static final int kCurrentLimit = 0; //int for some reason
    public static final double kDegree_per_rads = (360 / kGearRatio);
    public static final double kZero = 0.0;

    public static final double kMax_Vel = Units.degreesToRadians(960.0);
    public static final double kMax_Accel = Units.degreesToRadians(720.0);
    public static final Constraints kConstraints = new Constraints(kMax_Vel, kMax_Accel);

    public static final double kP = 4.0;
    public static final double kI = 0.3;
    public static final double kD = 0.2;
    public static final double kF = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kG = 1.0;


    public static final ProfiledPIDController WristPID = new ProfiledPIDController(kP, kI, kD, kConstraints);
    public static final ArmFeedforward WristFF = new ArmFeedforward(kS, kG, kV, kA);

    //sim
    public static final ProfiledPIDController SimWristPID = new ProfiledPIDController(3, 0.0, 6, new Constraints(Units.degreesToRadians(960.0), Units.degreesToRadians(720.0)));
    public static final ArmFeedforward SimWristFF = new ArmFeedforward(0.1, 0, 0, 0);

    
  }

  public static class IntakeConstants {
    public static double voltStall = Amp.of(257).magnitude(); //rpm threshold to consider for stalling

    private static final CurrentLimitsConfigs kCurrentLimitsConfigs = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(60)
      .withSupplyCurrentLimit(40)
      .withStatorCurrentLimitEnable(true);

      public static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration()
        .withCurrentLimits(kCurrentLimitsConfigs)
        .withMotorOutput(
          new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
        );
  }
}