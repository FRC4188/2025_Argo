package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

// TODO: complete simulation
public class ElevatorIOSim implements ElevatorIO {

  private static final double ELE_KP = 8.0;
  private static final double ELE_KD = 0.0;
  private static final DCMotor ELE_GEARBOX = DCMotor.getKrakenX60Foc(2);

  private final DCMotorSim eleSim;

  private boolean eleClosedLoop = false;
  private PIDController eleController = new PIDController(ELE_KP, 0, ELE_KD);
  private double eleAppliedVolts = 0.0;

  public ElevatorIOSim() {

    eleSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ELE_GEARBOX, 0.004, Constants.EleConstants.kGearRatio),
            ELE_GEARBOX);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (eleClosedLoop) {
      eleAppliedVolts = eleController.calculate(eleSim.getAngularPositionRad());
    } else {
      eleController.reset();
    }

    eleSim.setInputVoltage(MathUtil.clamp(eleAppliedVolts, -12.0, 12.0));
    eleSim.update(0.02);

    inputs.leaderConnected = true;
    inputs.followConnected = true;
    inputs.positionRad = eleSim.getAngularPositionRad();
    inputs.velocityRadPerSec = eleSim.getAngularVelocityRadPerSec();
    inputs.leaderAppliedVolts = eleAppliedVolts;
    inputs.leaderCurrentAmps = Math.abs(eleSim.getCurrentDrawAmps());
    inputs.followAppliedVolts = eleAppliedVolts;
    inputs.followCurrentAmps = Math.abs(eleSim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double output) {
    eleClosedLoop = false;
    eleAppliedVolts = output;
  }

  @Override
  public void setPosition(Rotation2d rotation) {
    eleClosedLoop = true;
    eleController.setSetpoint(rotation.getRadians());
  }
}
