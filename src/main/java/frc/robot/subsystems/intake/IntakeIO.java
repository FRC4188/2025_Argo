package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double velocityRotPerSec = 0.0;

    public boolean falconConnected = false;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the motors at the specified open loop value. */
  public default void setOpenLoop(double output) {}

  /** Run the intake to the specified rotations. */
  public default void setVelocity(double rotations) {}

  public default boolean isStalled() {
    return false;
  }

  /** Update the PID values of the elevator */
  public default void updatePID(double kP, double kI, double kD) {}
}
