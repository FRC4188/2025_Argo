package frc.robot.subsystems.tracking;

// import static frc.robot.subsystems.vision.VisConstants.*;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Tracking extends SubsystemBase {
  private final TrackingIO io;
  private final TrackingIOInputsAutoLogged inputs;
  private final Alert disconnectedAlert;

  public Tracking(TrackingIO io) {
    this.io = io;
    // Initialize inputs
    this.inputs = new TrackingIOInputsAutoLogged();
    disconnectedAlert = new Alert("Disconnected tracking vision camera", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Tracking", inputs);
  }

  @AutoLogOutput(key = "Tracking Vision/Has Target")
  public boolean hasTarget() {
    return inputs.hasTarget;
  }

  public Transform2d getTarget() {
    return inputs.targetTransform;
  }
}
