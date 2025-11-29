package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean wristConnected = false;
    public boolean cancoderConnected = false;

    public Rotation2d wristAbsolutePosition = new Rotation2d();
    public Rotation2d wristPosition = new Rotation2d();

    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setOpenLoop(double output) {}

  public default void setPosition(Rotation2d rotation) {}

  public default void updatePID(double kP, double kI, double kD, double kG) {}
}
