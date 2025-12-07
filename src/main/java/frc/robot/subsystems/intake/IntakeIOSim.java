package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

// TODO: complete sim
public class IntakeIOSim implements IntakeIO {
  private static final double KP = 0.05;
  private static final double KD = 0.0;
  private static final double KS = 0.0;
  private static final double KV_ROT = 0.91035;
  private static final double KV = 1.0 / Units.rotationsToRadians(1.0 / KV_ROT);
  private static final DCMotor GEARBOX = DCMotor.getFalcon500Foc(1);

  private final DCMotorSim intakeSim;

  private boolean intakeClosedLoop = false;
  private PIDController intakeController = new PIDController(KP, 0, KD);
  private double intakeFFVolts = 0.0;
  private double intakeAppliedVolts = 0.0;

  public IntakeIOSim() {
    intakeSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(GEARBOX, 1.0, 1.0), GEARBOX);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (intakeClosedLoop) {
      intakeAppliedVolts =
          intakeFFVolts + intakeController.calculate(intakeSim.getAngularVelocityRadPerSec());
    } else {
      intakeController.reset();
    }

    intakeSim.setInputVoltage(MathUtil.clamp(intakeAppliedVolts, -12.0, 12.0));
    intakeSim.update(0.02);

    inputs.falconConnected = true;
    inputs.velocityRotPerSec = intakeSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = intakeAppliedVolts;
    inputs.currentAmps = Math.abs(intakeSim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double output) {
    intakeClosedLoop = false;
    intakeAppliedVolts = output;
  }

  @Override
  public void setVelocity(double velocityRotPerSec) {
    intakeClosedLoop = true;
    intakeFFVolts =
        KS * Math.signum(2 * Math.PI * velocityRotPerSec) + KV * 2 * Math.PI * velocityRotPerSec;
    intakeController.setSetpoint(2 * Math.PI * velocityRotPerSec);
  }
}
