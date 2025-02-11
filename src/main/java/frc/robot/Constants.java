package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.MomentOfInertiaUnit;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

  public static class controller {
    public static final int PILOT = 0;
    public static final int COPILOT = 1;
    public static final double DEADBAND = 0.1;
    public static final double TRIGGER_THRESHOLD = 0.6;
  }

  public static class robot {
    public static final double loopPeriodSecs = 0.02;

    public static final Mode currMode = RobotBase.isReal()? Mode.REAL : Mode.SIM;

    public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(12.6);

    public static final double A_LENGTH = Units.inchesToMeters(30); //inches
    public static final double A_WIDTH = Units.inchesToMeters(29); //inches
    public static final double A_CROSSLENGTH = Math.hypot(A_LENGTH, A_WIDTH);

    public static  final PIDConstants DRIVE_PID = new PIDConstants(5.0, 0.0, 0.0);
    public static  final PIDConstants TURN_PID = new PIDConstants(5.0, 0.0, 0.0);

    public static final Matrix<N3, N1> STATE_STD_DEVS = VecBuilder.fill(0.05, 0.05, 0.001);
    public static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(0.020, 0.020, 0.264);

    public static enum STATE {
      EMPTY,
      ALGAE,
      CORAL
    }
  
    public static STATE robotstate = STATE.EMPTY; //to be set
  }

  public static enum Mode {
    REAL,
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  public class Id{
    //DT ids are 1->12
    public static final int kElevatorLead = 13;
    public static final int kElevatorFollow = 14;
    public static final int kElevatorLeadNcoder = 15;
    public static final int kElevatorFollowNcoder = 16;
    public static final int kArm = 17;
    public static final int kArmNcoder = 18;    
    public static final int kWrist = 19;
    public static final int kWristNcoder = 20;    
    public static final int kIntake = 21;
  }

  //TODO: wuts dis??
  public class mode{
    int real = 0; 
  }

  public static class ElevatorConstants{    
    public static final double kDrumeRadius = 0.0; //TODO: get drum radius

    public static final double kGearRatio = 6;
    public static final double kZero = 0; //TODO: get zero

    private static final CurrentLimitsConfigs kCurrentLimitsConfigs = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(100)
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
      .withKP(0.0)
      .withKD(0.0)
      .withKS(0)
      .withKV(0.0)
      .withKA(0.0);

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

    public static final double kMax_Vel = 960.0;
    public static final double kMax_Accel = 720.0;
    public static final Constraints kConstraints = new Constraints(kMax_Vel, kMax_Accel);

    public static final ProfiledPIDController WristPID = new ProfiledPIDController(0.325, 0.0, 0.02, kConstraints);
    
    public static final int kCurrentLimit = 0;
    public static final double kGearRatio = 4.6666666667;

    public static final double kDegree_per_rads = (360 / kGearRatio);

    public static final double kTolerance = 0.75;
    public static final double kZero = 0.0; //TODO: to be tuned

    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kF = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    
  }

  public static class ArmConstants {
    public static final double kGearRatio = 5.0625;
  }
  

}