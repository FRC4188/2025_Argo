package frc.robot.subsystems.scoring.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {
    private SparkMax motor;

    private final double appliedVolts;
    private final double tempC;
    // private final StatusSignal<Angle> posRads;
    // private final StatusSignal<AngularVelocity> velRadsPerSec;

    public IntakeIOReal(){
        motor = new SparkMax(Constants.Id.kIntake, MotorType.kBrushless);
        SparkMaxConfig sparkconfig = new SparkMaxConfig();
        //TODO: find actual stall limit
        sparkconfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake);

        // we probably need safe params but not necessarily persist params
        motor.configure(sparkconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        appliedVolts = motor.getOutputCurrent();
        tempC = motor.getMotorTemperature();
        // posRads = encoder.getPosition();
        // velRadsPerSec = encoder.getVelocity();

        //signal updates less frequently since intake is less important, and to reduce CAN bus traffic
        //intake acc doesnt need status signal at all, i included it for options so we can monitor voltage usage
    
    
    // "appliedVolts" and "tempC" both update automatically at 20.0 ms (50 hz)
    //     BaseStatusSignal.setUpdateFrequencyForAll(
    //         Frequency.ofRelativeUnits(10.0,
    //         Units.Hertz), 
    //         appliedVolts,
    //         tempC
    //         // posRads,
    //         // velRadsPerSec);
    //     );
        
    //     motor.optimizeBusUtilization();
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