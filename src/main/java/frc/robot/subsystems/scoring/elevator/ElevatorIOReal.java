package frc.robot.subsystems.scoring.elevator;

import static edu.wpi.first.units.Units.Hertz;
import static frc.robot.Constants.ElevatorConstants.kMotorConfig;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.Id;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO {

    private final TalonFX leader;
    private final TalonFX follower;

    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Temperature> tempC;
    private final StatusSignal<Angle> posRads;
    private final StatusSignal<Voltage> appliedVoltsFollow;
    private final StatusSignal<Temperature> tempCFollow;

    public ElevatorIOReal() {

        //TODO: Set all the device ids, 0 for now cause idk robot isnt built???
        leader = new TalonFX(Id.kElevatorLead, Constants.robot.rio);
        follower = new TalonFX(Id.kElevatorFollow, Constants.robot.rio);

        follower.setControl(new Follower(Id.kElevatorLead, false));

        leader.setNeutralMode(NeutralModeValue.Brake);
        follower.setNeutralMode(NeutralModeValue.Brake);

        leader.clearStickyFaults();
        follower.clearStickyFaults();

        leader.getConfigurator().apply(kMotorConfig);
        follower.getConfigurator().apply(kMotorConfig);

        leader.optimizeBusUtilization();
        follower.optimizeBusUtilization();    
        
        posRads = leader.getPosition();
        appliedVolts = leader.getMotorVoltage();
        tempC = leader.getDeviceTemp();
        appliedVoltsFollow = follower.getMotorVoltage();
        tempCFollow = follower.getDeviceTemp();

        posRads.setUpdateFrequency(Hertz.of(50));
        appliedVolts.setUpdateFrequency(Hertz.of(50));
        tempC.setUpdateFrequency(Hertz.of(0.5));
        appliedVoltsFollow.setUpdateFrequency(Hertz.of(50));
        tempCFollow.setUpdateFrequency(Hertz.of(0.5));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.connected =
        BaseStatusSignal.refreshAll(
                posRads, appliedVolts, tempC, appliedVoltsFollow, tempCFollow)
            .isOK();
    
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.tempC = tempC.getValueAsDouble();
        inputs.posMeters = (posRads.getValueAsDouble()) * ElevatorConstants.kConversion;
        
        inputs.followerAppliedVolts = appliedVoltsFollow.getValueAsDouble();
        inputs.followerTempC = tempCFollow.getValueAsDouble();
    }

    @Override
    public void runVolts(double volts){
        leader.setControl(new VoltageOut(volts));
    }

    @Override
    public double getHeight(){
        return (posRads.getValueAsDouble()) * ElevatorConstants.kConversion; //TODO: test da math
    }
}