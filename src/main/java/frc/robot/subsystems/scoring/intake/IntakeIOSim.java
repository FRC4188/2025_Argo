package frc.robot.subsystems.scoring.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim sim;
    private double appliedVolts = 0.0;

    //random values for moment of inertia and gearing cuz not that important for precision
    public IntakeIOSim() {
        sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNeo550(1), 
                18.0 / 12.0,
                0.001), 
            DCMotor.getFalcon500(1));
    }

    @Override
    public void runVolts(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        sim.setInputVoltage(appliedVolts);
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        runVolts(0.0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        if(DriverStation.isDisabled())
            runVolts(0.0);

        sim.update(Constants.robot.loopPeriodSecs);
        inputs.appliedVolts = appliedVolts;
        inputs.posRads = sim.getAngularPositionRad();
        inputs.velRadsPerSec = sim.getAngularVelocityRadPerSec();
    }   
    
}