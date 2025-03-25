package frc.robot.subsystems.scoring.wrist;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;
import frc.robot.Constants.WristConstants;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

public class WristIOReal implements WristIO {
    private final SparkMax max = new SparkMax(Constants.Id.kWrist, MotorType.kBrushless);
    private final CANcoder canCoder = new CANcoder(Constants.Id.kWristCANCoder);

    private final StatusSignal<Angle> posRots;

    public WristIOReal() {  
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(WristConstants.kCurrentLimit);
        config.closedLoopRampRate(0.01);
        config.signals
            .absoluteEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionAlwaysOn(true);

        
        max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        canCoder.getConfigurator().apply(cancoderConfig);

        posRots = canCoder.getAbsolutePosition();
        posRots.setUpdateFrequency(50);
    }

    @Override
    public void runVolts(double volts){
        volts = MathUtil.clamp(volts,-12, 12);
        max.setVoltage(volts);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        posRots.refresh();
        inputs.appliedVolts = max.getAppliedOutput() * max.getBusVoltage();
        inputs.tempC = max.getMotorTemperature();
        inputs.posRads = Units.rotationsToRadians(posRots.getValueAsDouble());
    }

    @Override
    public double getAngle() {
        return Units.rotationsToRadians(posRots.getValueAsDouble());
    }
}