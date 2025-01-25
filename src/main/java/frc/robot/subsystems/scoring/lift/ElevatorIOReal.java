package frc.robot.subsystems.scoring.lift;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.scoring.lift.ElevatorIO.ElevatorIOInputs;;

public class ElevatorIOReal implements ElevatorIO {

    private final TalonFX RightMotor;
    private final TalonFX LeftMotor;

    private final CANcoder RightEncoder;
    private final CANcoder LeftEncoder;

    private final StatusSignal<Voltage> appliedVoltsLeft;
    private final StatusSignal<Temperature> tempCLeft;
    private final StatusSignal<Angle> posRadsLeft;
    private final StatusSignal<AngularVelocity> velRadsPerSecLeft;
    private final StatusSignal<Voltage> appliedVoltsRight;
    private final StatusSignal<Temperature> tempCRight;
    private final StatusSignal<Angle> posRadsRight;
    private final StatusSignal<AngularVelocity> velRadsPerSecRight;
    private final StatusSignal<Angle> desiredPos;
    private final StatusSignal<AngularVelocity> desiredVel;

    public ElevatorIOReal() {

        //TODO: Set all the device ids, 0 for now cause idk robot isnt built???
        LeftMotor = new TalonFX(ElevatorConstants.kLeadID);
        LeftEncoder = new CANcoder(ElevatorConstants.kLeadCANID);
        RightMotor = new TalonFX(ElevatorConstants.kFollowID);
        RightEncoder = new CANcoder(ElevatorConstants.kFollowCANID);

        RightMotor.setControl(new Follower(ElevatorConstants.kLeadID, false));

        LeftMotor.setNeutralMode(NeutralModeValue.Brake);
        RightMotor.setNeutralMode(NeutralModeValue.Brake);

        LeftMotor.clearStickyFaults();
        RightMotor.clearStickyFaults();
        LeftEncoder.clearStickyFaults();
        RightEncoder.clearStickyFaults();

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

        RightMotor.getConfigurator().apply(ElevatorConstants.kMotorConfig);
        LeftMotor.getConfigurator().apply(ElevatorConstants.kMotorConfig);

        LeftMotor.optimizeBusUtilization();
        RightMotor.optimizeBusUtilization();    
        

        appliedVoltsLeft = LeftMotor.getMotorVoltage();
        tempCLeft = LeftMotor.getDeviceTemp();
        posRadsLeft = LeftMotor.getPosition();
        velRadsPerSecLeft = LeftEncoder.getVelocity();
        
        appliedVoltsRight = RightMotor.getMotorVoltage();
        tempCRight = RightMotor.getDeviceTemp();
        posRadsRight = RightMotor.getPosition();
        velRadsPerSecRight = RightEncoder.getVelocity();
        //TODO: Set these to do something
        desiredPos = RightEncoder.getAbsolutePosition();
        desiredVel = RightEncoder.getVelocity();
        
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.appliedVoltsLeft = appliedVoltsLeft.getValueAsDouble();
        inputs.tempCLeft = tempCLeft.getValueAsDouble();
        inputs.posRadsLeft = posRadsLeft.getValueAsDouble();
        inputs.velRadsPerSecLeft = velRadsPerSecLeft.getValueAsDouble();
        
        inputs.appliedVoltsRight = appliedVoltsRight.getValueAsDouble();
        inputs.tempCRight = tempCRight.getValueAsDouble();
        inputs.posRadsRight = posRadsRight.getValueAsDouble();
        inputs.velRadsPerSecRight = velRadsPerSecRight.getValueAsDouble();
        //TODO: Set these to do something
        inputs.desiredPos = posRadsRight.getValueAsDouble();
        inputs.desiredVel = posRadsRight.getValueAsDouble();
    }

    @Override
    public void runVolts(double volts){
        LeftMotor.setControl(new VoltageOut(volts));
        RightMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void runPosition(double height){
        LeftMotor.setControl(new PositionVoltage(height));
        RightMotor.setControl(new PositionVoltage(height));
    }
}
