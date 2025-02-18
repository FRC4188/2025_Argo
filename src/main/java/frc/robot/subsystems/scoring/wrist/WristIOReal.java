package frc.robot.subsystems.scoring.wrist;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;

import com.revrobotics.RelativeEncoder;

import static frc.robot.Constants.WristConstants.*;

public class WristIOReal implements WristIO {
    private final SparkMax max = new SparkMax(Constants.Id.kWrist, MotorType.kBrushless);
    private final RelativeEncoder encoder;

    private double appliedVolts;
    private double tempC;
    private double posRads;

    public WristIOReal(){
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true).idleMode(IdleMode.kBrake);
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(kCurrentLimit);
        config.absoluteEncoder.apply(new AbsoluteEncoderConfig().zeroOffset(kZero));

        max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        encoder = max.getEncoder();
    }

    @Override
    public void runVolts(double volts){
        volts = MathUtil.clamp(volts,-12, 12);
        max.setVoltage(volts);
    }

    @Override  
    public void stop() {
        runVolts(0);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        appliedVolts = max.getAppliedOutput() * max.getBusVoltage();
        tempC = max.getMotorTemperature();        
        posRads = encoder.getPosition() / (2*Math.PI);

        inputs.appliedVolts = appliedVolts;
        inputs.tempC = tempC;
        inputs.posRads = posRads;
    }

    @Override
    public double getAngle() {
        return posRads;
    }
}