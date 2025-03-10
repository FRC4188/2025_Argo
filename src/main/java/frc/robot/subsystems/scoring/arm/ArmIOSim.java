
// package frc.robot.subsystems.scoring.arm;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.simulation.DCMotorSim;
// import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
// import frc.robot.Constants;
// import frc.robot.subsystems.scoring.superstructure.SuperstructureConfig;
// import frc.robot.subsystems.scoring.superstructure.SuperConstraints.ArmConstraints;
// // This is definetly wrong but work in progress I think I got the wrong concept
// public class ArmIOSim implements ArmIO {
//     private final DCMotorSim sim;
//     private double appliedVolts = 0.0;
//     private SingleJointedArmSim armSim;
    
//     //random values for moment of inertia and gearing cuz not that important for precision
//     public ArmIOSim() {
        
//         sim = new DCMotorSim(
//             LinearSystemId.createDCMotorSystem(
//                 DCMotor.getFalcon500(1), 
//                 SuperstructureConfig.arm.inertiaAbtCoM(),
//                 Constants.ArmConstants.kGearRatio), 
//             DCMotor.getFalcon500(1));
        
//         armSim = new SingleJointedArmSim(
//             DCMotor.getFalcon500(1), 
//             Constants.ArmConstants.kGearRatio,
//             SuperstructureConfig.arm.inertiaAbtCoM(), 
//             SuperstructureConfig.arm.length(), 
//             ArmConstraints.LOWEST_A, 
//             ArmConstraints.HIGHEST_A,
//             true,
//              0.0
//             );
//     }

//     @Override
//     public void runVolts(double volts) {
//         appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);

//         sim.setInputVoltage(appliedVolts);
//         armSim.setInputVoltage(appliedVolts);
//     }

//     @Override
//     public void updateInputs(ArmIOInputs inputs) {
//         if(DriverStation.isDisabled()){
//             runVolts(0.0);
//         }
//         sim.update(Constants.robot.loopPeriodSecs);
//         inputs.appliedVolts = appliedVolts;
//         inputs.positionRads = armSim.getAngleRads();
//     } 

//     @Override
//     public double getAngle(){
//         return sim.getAngularPositionRad();
//     }
    
// }
