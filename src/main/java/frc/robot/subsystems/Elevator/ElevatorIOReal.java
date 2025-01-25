package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.Elevator.ElevatorIO.ElevatorIOInputs;;

public class ElevatorIOReal implements ElevatorIO {

    TalonFX RightMotor;
    TalonFX LeftMotor;

    CANcoder RightEncoder;
    CANcoder LeftEncoder;

    double LeftZero = 0;
    double RightZero = 0;

    ElevatorFeedforward PID;
    double speedAfterPID;

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
        LeftMotor = new TalonFX(0);
        LeftEncoder = new CANcoder(0);
        RightMotor = new TalonFX(0);
        RightEncoder = new CANcoder(0);

        //TODO: Tune this   
        PID = new ElevatorFeedforward(0.0, 0.0, 0.0);
        


        LeftMotor.setNeutralMode(NeutralModeValue.Brake);
        OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
        openLoopRampsConfigs.VoltageOpenLoopRampPeriod = 0.5;
        LeftMotor.getConfigurator().apply(openLoopRampsConfigs);
        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = 0.5;
        LeftMotor.getConfigurator().apply(closedLoopRampsConfigs);
        RightMotor.setNeutralMode(NeutralModeValue.Brake);
        OpenLoopRampsConfigs openLoopRampsConfigsAngle = new OpenLoopRampsConfigs();
        openLoopRampsConfigsAngle.VoltageOpenLoopRampPeriod = 0.5;
        RightMotor.getConfigurator().apply(openLoopRampsConfigs);
        ClosedLoopRampsConfigs closedLoopRampsConfigsAngle = new ClosedLoopRampsConfigs();
        closedLoopRampsConfigsAngle.VoltageClosedLoopRampPeriod = 0.5;
        RightMotor.getConfigurator().apply(closedLoopRampsConfigs);
        RightMotor.setInverted(true);

        LeftMotor.clearStickyFaults();
        RightMotor.clearStickyFaults();
        LeftEncoder.clearStickyFaults();
        RightEncoder.clearStickyFaults();

        MagnetSensorConfigs leftSensorConfigs = new MagnetSensorConfigs();
        leftSensorConfigs.MagnetOffset = -(LeftZero / 360.0);
        leftSensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        MagnetSensorConfigs rightSensorConfigs = new MagnetSensorConfigs();
        rightSensorConfigs.MagnetOffset = -(LeftZero / 360.0);
        rightSensorConfigs.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        LeftEncoder.getConfigurator().apply(leftSensorConfigs);
        LeftMotor.getConfigurator().setPosition(LeftEncoder.getAbsolutePosition().getValueAsDouble() * 360.0);

        RightEncoder.getConfigurator().apply(rightSensorConfigs);
        RightMotor.getConfigurator().setPosition(RightEncoder.getAbsolutePosition().getValueAsDouble() * 360.0);

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

    public void runVolts(double speed) {
        LeftMotor.set(speed);
        RightMotor.set(speed);
    }
    public void stop(){
        LeftMotor.set(0.0);
        RightMotor.set(0.0);
    }
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
}
