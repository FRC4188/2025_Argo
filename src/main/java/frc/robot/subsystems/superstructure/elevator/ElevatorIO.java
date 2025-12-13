package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;

    public boolean leaderConnected = false;
    public double leaderAppliedVolts = 0.0;
    public double leaderCurrentAmps = 0.0;

    public boolean followConnected = false;
    public double followAppliedVolts = 0.0;
    public double followCurrentAmps = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setOpenLoop(double output) {}

  public default void setZero() {}

  public default void setPosition(Rotation2d rotation) {}

  public default void updatePID(double kP, double kI, double kD, double kG) {}

  public default double getSetpoint() {
    return 0;
  }
}
