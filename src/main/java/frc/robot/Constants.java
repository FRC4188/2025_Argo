package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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
    public static final double DEADBAND = 0.1;
    public static final double TRIGGER_THRESHOLD = 0.6;
  }

  public static class robot {
    public static final double loopPeriodSecs = 0.02;

    public static final Mode simMode = Mode.SIM;
    public static final Mode currMode = RobotBase.isReal()? Mode.REAL : simMode;

    public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(12.6);

    public static final double A_LENGTH = Units.inchesToMeters(30); //inches
    public static final double A_WIDTH = Units.inchesToMeters(29); //inches
    public static final double A_CROSSLENGTH = Math.hypot(A_LENGTH, A_WIDTH);

    public static  final PIDConstants DRIVE_PID = new PIDConstants(5.0, 0.0, 0.0);
    public static  final PIDConstants TURN_PID = new PIDConstants(5.0, 0.0, 0.0);

    public static final Matrix<N3, N1> STATE_STD_DEVS = VecBuilder.fill(0.05, 0.05, 0.001);
    public static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(0.020, 0.020, 0.264);

  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public class mode{
    int real = 0;
    
  }

  public static class ids{
    public static final int INTAKE = 16;

    public static final int SHOULDER_LEADER = 17;
    public static final int SHOULDER_FOLLOWER = 18;

    public static final int LEFT_SHOOTER = 19;
    public static final int RIGHT_SHOOTER = 20;

    public static final int LEFT_CLIMBER = 21;
    public static final int RIGHT_CLIMBER = 22;

    public static final int FEEDER = 23;

    public static final int SHOULDER_ENCODER = 24;

    public static final int FEEDER_BEAM_BREAKER = 5; // input = 0
    
    public static final int INTAKE_BEAM_BREAKER_1 = 8;
    public static final int INTAKE_BEAM_BREAKER_2 = 9;

    public static final int CLIMBER_LEFT_LIMIT = 4;
    public static final int CLIMBER_RIGHT_LIMIT = 1;


  }
  public static class wrist {

    public static final double MAX_VEL = 960.0;
    public static final double MAX_ACCEL = 720.0;
    public static final Constraints CONSTRAINTS = new Constraints(MAX_VEL, MAX_ACCEL);

    public static final ProfiledPIDController WristPID = new ProfiledPIDController(0.325, 0.0, 0.02, CONSTRAINTS);
    
    public static final int WRIST = 24;
    public static final int WRIST_ENCODER = 10;

    public static final float WRIST_SOFT_LIMIT = 0.0f;
    public static final double WRIST_GEAR_RATIO = 4.6666666667;

    public static final double WRIST_DEGREES_PER_MOTOR_ROTATION = (360 / WRIST_GEAR_RATIO);
    public static final float WRIST_OUT_SOFT_LIMIT = 111f;
    public static final float WRIST_ELEVATOR_OUT_SOFT_LIMIT = 270;

    public static final double MAX_TEMP = 50.0;
    public static final double MAX_TEMP_WARNING = 60.0;

    public static final double UPPER_LIMIT = 117.0;
    public static final double LOWER_LIMIT = -117.0;

    public static final double ALLOWED_ERROR = 0.75;

    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kF = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    
  }
  

}
