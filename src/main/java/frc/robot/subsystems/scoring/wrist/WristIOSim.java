package frc.robot.subsystems.scoring.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.scoring.superstructure.SuperstructureConfig;
import frc.robot.subsystems.scoring.superstructure.SuperConstraints.WristConstraints;

public class WristIOSim implements WristIO{
    private final DCMotorSim sim;
    private double appliedVolts = 0.0;
    private SingleJointedArmSim wSim;

    public WristIOSim() {
        sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNeo550(1), 
                SuperstructureConfig.wrist.inertiaAbtCoM(), 
                Constants.ArmConstants.kGearRatio), 
            DCMotor.getNeo550(1));

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


    public ProfiledPIDController getPID() {
        return WristConstants.WristPID;
    }

    public ArmFeedforward getFF() {
        return WristConstants.WristFF;
    }
    
    @Override
    public void runVolts(double volts) {
        appliedVolts = MathUtil.clamp(volts,-12, 12);
        sim.setInputVoltage(appliedVolts);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        if(DriverStation.isDisabled()){
            runVolts(0.0);
        }
        sim.update(Constants.robot.loopPeriodSecs);
        inputs.appliedVolts = appliedVolts;
        inputs.posRads = wSim.getAngleRads();
    }

    @Override
    public double getAngle() {
        return sim.getAngularPositionRad();
    }
}