package frc.robot.subsystems.scoring.arm;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
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

        CurrentLimitsConfigs kCurrentLimitsConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(100)
        .withSupplyCurrentLimit(60)
        .withStatorCurrentLimitEnable(true);

         FeedbackConfigs FeedbackConfigs = new FeedbackConfigs()
        .withSensorToMechanismRatio(5.0625);
    
         MotionMagicConfigs MagicConfigs = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(RotationsPerSecond.of(1))
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

        Slot0Configs kSlot0Configs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKP(0.0)
        .withKD(0.0)
        .withKS(0)
        .withKV(0.0)
        .withKA(0.0);

        
        OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
        openLoopRampsConfigs.VoltageOpenLoopRampPeriod = 0.5;
        armMotor.getConfigurator().apply(openLoopRampsConfigs);
        
        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = 0.5;
        armMotor.getConfigurator().apply(closedLoopRampsConfigs);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration()
        .withCurrentLimits(kCurrentLimitsConfigs)
        .withFeedback(FeedbackConfigs)
        .withMotionMagic(MagicConfigs)
        .withSlot0(kSlot0Configs)
        .withClosedLoopRamps(closedLoopRampsConfigs)
        .withOpenLoopRamps(openLoopRampsConfigs);

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