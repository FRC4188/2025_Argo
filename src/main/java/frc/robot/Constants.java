package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
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

  public static class arm{



  }


}
