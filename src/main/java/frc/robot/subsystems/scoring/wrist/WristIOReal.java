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

public class WristIOReal implements WristIO {//J.C
    private SparkMax max = new SparkMax(1, MotorType.kBrushless);
    private RelativeEncoder encoder;

    private final double appliedVolts;
    private final double tempC;
    private final double posRads;
    private final double velRadsPerSec;

    public WristIOReal(){


        max = new SparkMax(Constants.Id.kWrist, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true).idleMode(IdleMode.kBrake);
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(kCurrentLimit);
        config.absoluteEncoder.apply(new AbsoluteEncoderConfig().zeroOffset(kZero));


        max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        appliedVolts = max.getAppliedOutput() * max.getBusVoltage();
        tempC = max.getMotorTemperature();
        encoder = max.getEncoder();

        posRads = encoder.getPosition();
        velRadsPerSec = encoder.getVelocity();
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.appliedVolts = appliedVolts;
        inputs.tempC = tempC;
        inputs.posRads = posRads;
        inputs.velRadsPerSec = velRadsPerSec;
    }

    @Override
    public void runVolts(double volts){
        volts = MathUtil.clamp(volts,-12, 12);
        max.setVoltage(volts);
    }
}