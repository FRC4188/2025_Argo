package frc.robot.subsystems.scoring.elevator;

import static frc.robot.Constants.ElevatorConstants.kDrumeRadius;
import static frc.robot.Constants.ElevatorConstants.kMotorConfig;
import static frc.robot.Constants.ElevatorConstants.kZero;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.Id;

public class ElevatorIOReal implements ElevatorIO {

    private final TalonFX leader;
    private final TalonFX follower;

    private final CANcoder leadNcoder;
    private final CANcoder followNcoder;

    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Temperature> tempC;
    private final StatusSignal<Angle> posRads;
    private final StatusSignal<AngularVelocity> velRadsPerSec;
    private final StatusSignal<Voltage> appliedVoltsFollow;
    private final StatusSignal<Temperature> tempCFollow;

    public ElevatorIOReal() {

        //TODO: Set all the device ids, 0 for now cause idk robot isnt built???
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

        // MagnetSensorConfigs leftSensorConfigs = new MagnetSensorConfigs();
        // leftSensorConfigs.MagnetOffset = constants.kLOffset;
        // leftSensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        // MagnetSensorConfigs rightSensorConfigs = new MagnetSensorConfigs();
        // rightSensorConfigs.MagnetOffset = constants.kROffset;
        // rightSensorConfigs.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        // LeftEncoder.getConfigurator().apply(e);
        // LeftMotor.getConfigurator().setPosition(LeftEncoder.getAbsolutePosition().getValueAsDouble() * 360.0);

        // RightEncoder.getConfigurator().apply(rightSensorConfigs);
        // RightMotor.getConfigurator().setPosition(RightEncoder.getAbsolutePosition().getValueAsDouble() * 360.0);

        leader.getConfigurator().apply(kMotorConfig);
        follower.getConfigurator().apply(kMotorConfig);

        leader.optimizeBusUtilization();
        follower.optimizeBusUtilization();    
        
        posRads = leader.getPosition();
        appliedVolts = leader.getMotorVoltage();
        tempC = leader.getDeviceTemp();
        velRadsPerSec = leader.getVelocity();
        appliedVoltsFollow = follower.getMotorVoltage();
        tempCFollow = follower.getDeviceTemp();
        
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.connected =
        BaseStatusSignal.refreshAll(
                posRads, appliedVolts, tempC, velRadsPerSec, appliedVoltsFollow, tempCFollow)
            .isOK();
    
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.tempC = tempC.getValueAsDouble();
        inputs.posRads = posRads.getValueAsDouble();
        
        inputs.followerAppliedVolts = appliedVoltsFollow.getValueAsDouble();
        inputs.followerTempC = tempCFollow.getValueAsDouble();
    }

    @Override
    public void runVolts(double volts){
        leader.setControl(new VoltageOut(volts));
    }

    @Override
    public void stop() {
        runVolts(0);
    }
    
    @Override
    public double getHeight(){
        return posRads.getValueAsDouble() - kZero / 6 * kDrumeRadius * 2; //TODO: test da math
    }
}