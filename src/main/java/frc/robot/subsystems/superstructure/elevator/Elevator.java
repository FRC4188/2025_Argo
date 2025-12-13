package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Alert leaderDisconnectedAlert;
  private final Alert followerDisconnectedAlert;

  public Elevator(ElevatorIO io) {
    this.io = io;

    leaderDisconnectedAlert = new Alert("Disconnected leader motor.", AlertType.kError);
    followerDisconnectedAlert = new Alert("Disconnected follower motor.", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    leaderDisconnectedAlert.set(!inputs.leaderConnected);
    followerDisconnectedAlert.set(!inputs.followConnected);
  }

  public void setHeight(double height) {
    Logger.recordOutput("Elevator/SetPoint", height);
    height = MathUtil.clamp(height, 0, Constants.EleConstants.RANGE);

    io.setPosition(new Rotation2d(height / Constants.EleConstants.kConversion));
  }

  public void runVolts(double output) {
    output = MathUtil.clamp(output, -12.0, 12.0);
    io.setOpenLoop(output);
  }

  public void setZero() {
    io.setZero();
  }

  public void stop() {
    io.setOpenLoop(0.0);
  }

  @AutoLogOutput(key = "Elevator/At Height")
  public boolean atHeight() {
    return (Math.abs(getPositionMeters() - io.getSetpoint()) <= Constants.EleConstants.HEIGHT_TOL);
  }

  @AutoLogOutput(key = "Elevator/Height Meters")
  public double getPositionMeters() {
    return inputs.positionRad * Constants.EleConstants.kConversion;
  }

  public double getVelocityMetersPerSec() {
    return inputs.velocityRadPerSec * Constants.EleConstants.kConversion;
  }

  public void updatePID(double kp, double ki, double kd, double kg) {
    io.updatePID(kp, ki, kd, kg);
  }
}
