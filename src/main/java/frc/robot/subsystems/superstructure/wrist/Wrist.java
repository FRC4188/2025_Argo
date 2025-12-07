package frc.robot.subsystems.superstructure.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Wrist {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private final Alert wristDisconnectedAlert;

  public Wrist(WristIO io) {
    this.io = io;
    wristDisconnectedAlert = new Alert("Disconnected Wrist", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);

    wristDisconnectedAlert.set(!inputs.wristConnected);
  }

  public void setPosition(Rotation2d angle) {
    angle = Rotation2d.fromRadians(MathUtil.clamp(angle.getRadians(), 0, Math.PI / 2));

    Logger.recordOutput("Wrist/SetPoint", angle);
    io.setPosition(angle);
  }

  public void runVolts(double output) {
    output = MathUtil.clamp(output, -12.0, 12.0);
    io.setOpenLoop(output);
  }

  public void stop() {
    io.setOpenLoop(0.0);
  }

  public void updatePID(double kp, double ki, double kd, double kg) {
    io.updatePID(kp, ki, kd, kg);
  }

  public Rotation2d getAngle() {
    return inputs.wristPosition;
  }
}
