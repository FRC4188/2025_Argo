package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static class controller {
    public static final int PILOT = 0;
    public static final int COPILOT = 1;
    public static final double DEADBAND = 0.1;
    public static final double TRIGGER_THRESHOLD = 0.6;
  }

  public static class robot {
    public static final double A_SIDE = Units.inchesToMeters(30); //inches
    public static final double A_CROSSLENGTH = Math.hypot(A_SIDE, A_SIDE);

    public static final Matrix<N3, N1> STATE_STD_DEVS = VecBuilder.fill(0.05, 0.05, 0.001);
    public static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(0.020, 0.020, 0.264);

  }


}
