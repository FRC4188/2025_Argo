
package frc.robot.subsystems.scoring.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.scoring.SuperstructureConfig;
import frc.robot.subsystems.scoring.SuperstructureConfig;
// This is definetly wrong but work in progress I think I got the wrong concept
public class ArmIOSim implements ArmIO {
    private final DCMotorSim sim;
    private double appliedVolts = 0.0;
    private SingleJointedArmSim armSim;
    
    
    //random values for moment of inertia and gearing cuz not that important for precision
    public ArmIOSim() {
        
        sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getFalcon500(1), 
                SuperstructureConfig.arm.inertiaAbtCoM(),
                Constants.ArmConstants.kGearRatio), 
            DCMotor.getFalcon500(1));
        
        armSim = new SingleJointedArmSim(
            DCMotor.getFalcon500(1), 
            Constants.ArmConstants.kGearRatio,
            SuperstructureConfig.arm.inertiaAbtCoM(), 
            SuperstructureConfig.arm.length(), 
            Rotation2d.fromDegrees(0.0).getRadians(), 
            Rotation2d.fromDegrees(0.0).getRadians(),
            true,
             0.0, 
             0.0
            );
    }

    @Override
    public void runVolts(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        sim.setInputVoltage(appliedVolts);
        armSim.setInputVoltage(appliedVolts);
    }

    @Override
    public void stop() {
        runVolts(0.0);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        if(DriverStation.isDisabled()){
            runVolts(0.0);
        }
        sim.update(Constants.robot.loopPeriodSecs);
        inputs.appliedVolts = appliedVolts;
        inputs.positionRads = sim.getAngularPositionRad();
        inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    } 

    
    public double getArmAngle(){
        return armSim.getAngleRads() * (Math.PI / 180);
    }
    
    public static enum Mode{
        L1, L2_3, L4;
    }
    
}
