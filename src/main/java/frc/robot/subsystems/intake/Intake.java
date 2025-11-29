package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final Alert falconDisconnectedAlert;

  public Intake(IntakeIO io) {
    this.io = io;

    falconDisconnectedAlert = new Alert("Disconnected falcon motor.", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    falconDisconnectedAlert.set(!inputs.falconConnected);
  }

  public void setVelocityRPM(double rpm) {
    io.setVelocity(rpm / 60.0);
  }

  public void runVolts(double output) {
    io.setOpenLoop(output);
  }

  @AutoLogOutput(key = "Intake/Is Stalled?")
  public boolean isStalled() {
    return io.isStalled();
  }

  public void stop() {
    io.setOpenLoop(0.0);
  }

  @AutoLogOutput(key = "Intake/RPM")
  public double getRPM() {
    return inputs.velocityRotPerSec * 60.0;
  }

  public void updatePID(double kp, double ki, double kd, double kg) {
    io.updatePID(kp, ki, kd);
  }
}
