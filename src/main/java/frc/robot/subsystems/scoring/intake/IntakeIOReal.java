package frc.robot.subsystems.scoring.intake;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
    private WPI_TalonSRX motor;

    private final double appliedVolts;
    private final double tempC;
    // private final StatusSignal<Angle> posRads;
    // private final StatusSignal<AngularVelocity> velRadsPerSec;

    private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0.0);
    private final NeutralOut neutralOut = new NeutralOut();

    public IntakeIOReal(){
        // motor = new SparkMax(Constants.ids.INTAKE, MotorType.kBrushless);
        motor = new WPI_TalonSRX(Constants.ids.INTAKE);
        TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
        motor.setSafetyEnabled(true);
        //TODO: when do we need to check if safety is enabled???????????
        appliedVolts = motor.getMotorOutputVoltage();
        tempC = motor.getTemperature();
        // posRads = encoder.getPosition();
        // velRadsPerSec = encoder.getVelocity();

        
        // motor.optimizeBusUtilization();
    }


    @Override
    public void runVolts(double volts) {
        // motor.set(voltageOut.withOutput(volts));
        motor.setVoltage(volts);
    }

    @Override
    public void stop() {
        runVolts(0.0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedVolts = appliedVolts;
        inputs.tempC = tempC;
        // inputs.posRads = posRads.getValueAsDouble();
        // inputs.velRadsPerSec = velRadsPerSec.getValueAsDouble();
    }
    
}