
package frc.robot.subsystems.scoring.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Id;

import static frc.robot.Constants.*;

public class ElevatorIOSim implements ElevatorIO{
    private final DCMotorSim sim;
    private final DCMotor gearbox;
    private final PIDController controller = new PIDController((12.0 / 483.0) * 3, 0, 0);
    private double appliedVolts = 0.0;

    private final TalonFX leader;
    private final TalonFX follower;

    private final CANcoder leadNcoder;
    private final CANcoder followNcoder;

    public ElevatorIOSim(){
        gearbox = DCMotor.getFalcon500(2);
        sim = new DCMotorSim(
            LinearSystemId.createElevatorSystem(
                gearbox,
                0, 
                0, 
                0), 
            gearbox);

        sim.setState(0, 0); //maxlengthmeter / 2 , 0

        leader = new TalonFX(Id.kElevatorLead);
        leadNcoder = new CANcoder(Id.kElevatorLeadNcoder);
        follower = new TalonFX(Id.kElevatorFollow);
        followNcoder = new CANcoder(Id.kElevatorFollowNcoder);

        follower.setControl(new Follower(Id.kElevatorLead, false));

        leader.setNeutralMode(NeutralModeValue.Brake);
        follower.setNeutralMode(NeutralModeValue.Brake);

        leadNcoder.clearStickyFaults();
        leader.clearStickyFaults();
        follower.clearStickyFaults();
        followNcoder.clearStickyFaults();

        leader.getConfigurator().apply(ElevatorConstants.kMotorConfig);
        follower.getConfigurator().apply(ElevatorConstants.kMotorConfig);

        leader.optimizeBusUtilization();
        follower.optimizeBusUtilization();    
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        sim.update(robot.loopPeriodSecs);

        inputs.connected = true;
        inputs.appliedVolts = appliedVolts;
        inputs.posRads = sim.getAngularPositionRad();
        inputs.velRadsPerSec = sim.getAngularVelocityRadPerSec();
        
        inputs.followerAppliedVolts = appliedVolts;
    }

    @Override
    public void runVolts(double volts){
        appliedVolts = MathUtil.clamp(controller.calculate(sim.getInputVoltage(), volts), -12.0, 12.0);
        sim.setInputVoltage(appliedVolts);
    }

    @Override
    public void runPosition(double height, double ff){
        leader.setControl(new PositionVoltage(height).withFeedForward(ff));
        sim.setInputVoltage(leader.getSimState().getMotorVoltage());
    }
    
    
}
