package frc.robot.subsystems.scoring.wrist;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;
import frc.robot.Constants.WristConstants;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

public class WristIOReal implements WristIO {
    private final SparkMax max = new SparkMax(Constants.Id.kWrist, MotorType.kBrushless);

    public WristIOReal() {  
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.idleMode(IdleMode.kBrake);
        //config.smartCurrentLimit(WristConstants.kCurrentLimit);
        config.signals
            .absoluteEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionAlwaysOn(true);
        max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void runVolts(double volts){
        volts = MathUtil.clamp(volts,-12, 12);
        max.setVoltage(volts);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.appliedVolts = max.getAppliedOutput() * max.getBusVoltage();
        inputs.tempC = max.getMotorTemperature();
        inputs.posRads = Units.rotationsToRadians(max.getAbsoluteEncoder().getPosition()* WristConstants.kGearRatio);
    }

    @Override
    public double getAngle() {
        return Units.rotationsToRadians(max.getEncoder().getPosition() * WristConstants.kGearRatio);
    }
}