package frc.robot.subsystems.scoring.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.scoring.superstructure.SuperstructureConfig;
import frc.robot.subsystems.scoring.superstructure.SuperConstraints.WristConstraints;

public class WristIOSim implements WristIO{
    private double appliedVolts = 0.0;
    private final SingleJointedArmSim wSim;

    public WristIOSim() {

        wSim = new SingleJointedArmSim(
            DCMotor.getNeo550(1),
            Constants.WristConstants.kGearRatio,
            SuperstructureConfig.wrist.inertiaAbtCoM(), 
            SuperstructureConfig.wrist.length(), 
            WristConstraints.LOWEST_A + Math.PI/2, 
            WristConstraints.HIGHEST_A + Math.PI/2,
            true,
             Math.PI/2
            );
    }
    
    @Override
    public void runVolts(double volts) {
        appliedVolts = MathUtil.clamp(volts,-12, 12);
        wSim.setInputVoltage(volts);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        if(DriverStation.isDisabled()){
            runVolts(0.0);
        }

        wSim.update(0.02);
        inputs.appliedVolts = appliedVolts;
        inputs.posRads = wSim.getAngleRads();
    }

    @Override
    public double getAngle() {
        return wSim.getAngleRads() - Math.PI/2;
    }
}