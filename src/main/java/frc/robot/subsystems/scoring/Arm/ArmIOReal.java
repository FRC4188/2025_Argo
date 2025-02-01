package frc.robot.subsystems.scoring.Arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ArmIOReal implements ArmIO {
    TalonFX armMotor;
    CANcoder armEncoder;
    double armZero = 0;

    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Temperature> tempC;
    private final StatusSignal<Angle> posRads;
    private final StatusSignal<AngularVelocity> velRadsPerSec;
    private final StatusSignal<Angle> desiredPos;
    private final StatusSignal<AngularVelocity> desiredVel;

    public ArmIOReal() {
        //TODO: Set device IDs
        armMotor = new TalonFX(0);
        armEncoder = new CANcoder(0);

        armMotor.setNeutralMode(NeutralModeValue.Brake);
        
        OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
        openLoopRampsConfigs.VoltageOpenLoopRampPeriod = 0.5;
        armMotor.getConfigurator().apply(openLoopRampsConfigs);
        
        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = 0.5;
        armMotor.getConfigurator().apply(closedLoopRampsConfigs);

        armMotor.clearStickyFaults();
        armEncoder.clearStickyFaults();

        MagnetSensorConfigs sensorConfigs = new MagnetSensorConfigs();
        sensorConfigs.MagnetOffset = -(armZero / 360.0);
        sensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        armEncoder.getConfigurator().apply(sensorConfigs);
        armMotor.getConfigurator().setPosition(armEncoder.getAbsolutePosition().getValueAsDouble() * 360.0);

        armMotor.optimizeBusUtilization();

        appliedVolts = armMotor.getMotorVoltage();
        tempC = armMotor.getDeviceTemp();
        posRads = armMotor.getPosition();
        velRadsPerSec = armEncoder.getVelocity();
        desiredPos = armEncoder.getAbsolutePosition();
        desiredVel = armEncoder.getVelocity();
    }

    public void runVolts(double volts) {
        armMotor.set(volts);
    }

    public void stop() {
        armMotor.set(0.0);
    }

    public void updateInputs(ArmIOInputs inputs) {
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.tempC = tempC.getValueAsDouble();
        inputs.positionRads = posRads.getValueAsDouble();
        inputs.velocityRadPerSec = velRadsPerSec.getValueAsDouble();
        inputs.desiredPositionRads = desiredPos.getValueAsDouble();
        inputs.desiredVelocityRadPerSec = desiredVel.getValueAsDouble();
    }
}