package frc.robot.subsystems.scoring.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.scoring.SuperConstraints;
import frc.robot.subsystems.scoring.SuperConstraints.ArmConstraints;
import frc.robot.subsystems.scoring.SuperConstraints.WristConstraints;
import frc.robot.subsystems.scoring.SuperstructureConfig;

public class WristIOSim implements WristIO{//J.C

    private final DCMotorSim sim;
    private double appliedVolts = 0.0;
    private SingleJointedArmSim wSim;

    public WristIOSim() {
        var plant = LinearSystemId.createDCMotorSystem
            (DCMotor.getNeo550(1), 1, 1);
        var gearbox = DCMotor.getNeo550(1);

        sim = new DCMotorSim(plant, gearbox);

        wSim = new SingleJointedArmSim(
            DCMotor.getFalcon500(1), 
            Constants.WristConstants.kGearRatio,
            SuperstructureConfig.wrist.inertiaAbtCoM(), 
            SuperstructureConfig.wrist.length(), 
            WristConstraints.LOWEST_A, 
            WristConstraints.HIGHEST_A,
            true,
             0.0
            );
    }

    @Override
    public void runVolts(double volts) {
        appliedVolts = MathUtil.clamp(volts,-12, 12);
        sim.setInputVoltage(appliedVolts);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        sim.update(Constants.robot.loopPeriodSecs);
        inputs.appliedVolts = appliedVolts;
        inputs.posRads = wSim.getAngleRads();
        inputs.velRadsPerSec = sim.getAngularVelocityRadPerSec();
    }
}