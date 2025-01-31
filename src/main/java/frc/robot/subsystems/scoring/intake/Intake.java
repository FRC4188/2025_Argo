package frc.robot.subsystems.scoring.intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{

    //no getters for telemetry cuz thats wut the autologged inputs do
    private static Intake instance;
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;
    private static int safeParams;

    private SparkMax motor = new SparkMax(Constants.ids.INTAKE, MotorType.kBrushless);
    SparkMaxConfig sparkconfig = new SparkMaxConfig();

    public static Intake getInstance(IntakeIO io){
        if(instance == null){
            instance = new Intake(io);
        }
        return instance;
    }

    private Intake(IntakeIO io){
        this.io = io;
        inputs = new IntakeIOInputsAutoLogged();
    }

    public void runVolts(double volts){
        io.runVolts(volts);
    }

    public void stop(){
        io.stop();
    }

    public ResetMode disableSafeParameters() {
        ResetMode safeParams = ResetMode.kNoResetSafeParameters;
        return safeParams;
    }

    public ResetMode activateSafeParameters() {
        ResetMode safeParams = ResetMode.kResetSafeParameters;
        return safeParams;
    }


    public Command ingest(Intake intake) {
        return Commands.run(
            () -> {
                if () {}
                // motor.setInverted(false);
                sparkconfig.inverted(false);
                runVolts(10);
                
            }, intake);
    }

    public Command eject(Intake intake) {
        return Commands.run(
            () -> {
                // motor.setInverted(true);
                sparkconfig.inverted(true);
                runVolts(10);
            }, intake);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);    
    }
}
