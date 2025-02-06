package frc.robot.subsystems.scoring.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.scoring.intake.IntakeIO.IntakeIOInputs;

public class IntakeWristIOSim implements IntakeWristIO{//J.C
    int degreesL1 = 90;
    int degreesL2 = 110;
    int degreesL3 = 130;
    int degreesL4 = 150;//none of these are actual numbers

    private final DCMotorSim sim;
    private double appliedVolts = 0.0;

    public IntakeWristIOSim() {
        sim = new DCMotorSim(LinearSystemId.createDCMotorSystem
            (DCMotor.getNeo550(1), 1, 1), 
            DCMotor.getNeo550(1));
            //i don't know the SparkMax motor so i just looked one up 
    }

    @Override
    public void runVolts(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        sim.setInputVoltage(appliedVolts);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        sim.update(Constants.robot.loopPeriodSecs);
        inputs.appliedVolts = appliedVolts;
        inputs.posRads = sim.getAngularPositionRad();
        inputs.velRadsPerSec = sim.getAngularVelocityRadPerSec();
    }

    public static enum Mode{
        L1, L2_3, L4;
    }
}
