package frc.robot.subsystems.scoring.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged.Naming;
import edu.wpi.first.units.FrequencyUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.scoring.wrist.IntakeWristIO;


public class IntakeWristIOReal implements IntakeWristIO {//J.C
    private SparkMax max = new SparkMax(1, MotorType.kBrushless);

    private final double appliedVolts;
    private final double tempC;
    private final double posRads;
    private final double velRadsPerSec;

    private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
    private final NeutralOut neutralOut = new NeutralOut();

    public IntakeWristIOReal(){


        SparkMax max = new SparkMax(1, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true).idleMode(IdleMode.kBrake);
        config.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0, 0.0, 0.0);

        max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        appliedVolts = max.getAppliedOutput() * max.getBusVoltage();
        tempC = max.getMotorTemperature();
        RelativeEncoder encoder = max.getEncoder();

        posRads = max.configAccessor.encoder.getPositionConversionFactor();
        velRadsPerSec = max.configAccessor.encoder.getVelocityConversionFactor();
    }

    
    @Override
    public void updateInputs(IntakeWristIOInputs inputs) {
        inputs.appliedVolts = appliedVolts;
        inputs.tempC = tempC;
        inputs.posRads = posRads;
        inputs.velRadsPerSec = velRadsPerSec;
    }

}