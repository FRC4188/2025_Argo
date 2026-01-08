package frc.robot.subsystems.tracking;

import edu.wpi.first.math.geometry.Transform2d;
import org.littletonrobotics.junction.AutoLog;

public interface TrackingIO {
  @AutoLog
  public static class TrackingIOInputs {
    public boolean camConnected = false;
    public boolean hasTarget = false;
    public Transform2d targetTransform = new Transform2d();
  }

  public default void updateInputs(TrackingIOInputs inputs) {}
}
