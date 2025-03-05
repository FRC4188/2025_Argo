
package frc.robot.subsystems.scoring.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.robot;
import frc.robot.subsystems.scoring.Superstructure.SuperConstraints.ElevatorConstraints;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Pounds;

public class ElevatorIOSim implements ElevatorIO{
    private final DCMotorSim sim;
    private final DCMotor gearbox;
    private double appliedVolts = 0.0;

    private final ElevatorSim physSim;

    public ElevatorIOSim(){
        gearbox = DCMotor.getFalcon500(2);
        var plant =  LinearSystemId.createElevatorSystem(
            gearbox,
            Kilograms.convertFrom(23.37, Pounds), 
            ElevatorConstants.kDrumeRadius, 
            ElevatorConstants.kGearRatio);
        sim = new DCMotorSim(
            plant, 
            gearbox);

        sim.setState(0, 0); //maxlengthmeter / 2 , 0

        physSim = new ElevatorSim(
            plant, gearbox, 0, ElevatorConstraints.RANGE, true, 0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        
        inputs.connected = true;
        inputs.appliedVolts = appliedVolts;
        inputs.posRads = sim.getAngularPositionRad();
        
        inputs.followerAppliedVolts = appliedVolts;
        sim.update(robot.loopPeriodSecs);
        physSim.update(robot.loopPeriodSecs);
    }

    @Override
    public void runVolts(double volts){
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        sim.setInputVoltage(appliedVolts);
        physSim.setInputVoltage(volts);
    }

   
    public void stop() {
        runVolts(0);
    }

    @Override
    public double getHeight(){
        return physSim.getPositionMeters();
    }
    
    
}