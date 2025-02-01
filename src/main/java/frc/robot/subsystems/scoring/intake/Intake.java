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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{

    //no getters for telemetry cuz thats wut the autologged inputs do
    private static Intake instance;
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;

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

    public static enum GamePieceType {
        CORAL,
        ALGAE
    }

    public void runVolts(double volts){
        io.runVolts(volts);
    }

    public void stop(){
        io.stop();
    }

    public boolean intakeVoltageSpike() {
        if (motor.getAppliedOutput() > 5) {
            return true;
        } else {
            return false;
        }
    }

    // public ResetMode disableSafeParameters(ResetMode safeParams) {
    //     safeParams = ResetMode.kNoResetSafeParameters;
    //     return safeParams;
    // }

    // public ResetMode activateSafeParameters(ResetMode safeParams) {
    //     safeParams = ResetMode.kResetSafeParameters;
    //     return safeParams;
    // }


    //TODO: fix inverted for coral/algae ingest/eject (don't know which is inverted and which one isn't)
    public Command ingest(Intake intake) {
        return Commands.run(
            () -> {
                GamePieceType type = GamePieceType.CORAL;
                switch (type) {
                    case CORAL:
                        sparkconfig.inverted(false);
                        break;
                    case ALGAE:
                        sparkconfig.inverted(true);
                        break;
                }
                runVolts(1);
            }, intake);
    }

    public Command eject(Intake intake) {
        return Commands.run(
            () -> {
                GamePieceType type = GamePieceType.CORAL;
                switch (type) {
                    case CORAL:
                        sparkconfig.inverted(true);
                        break;
                    case ALGAE:
                        sparkconfig.inverted(false);
                        break;
                }
                sparkconfig.inverted(true);
                runVolts(1);
            }, intake);
    }

    public Command halt(Intake intake) {
        return Commands.run(
            () -> {
                stop();
            }, intake);
    }

    public Command detectIntakeSpike(Intake intake) {
        return Commands.run(
            () -> {
                intakeVoltageSpike();
            }, intake);
    }

    public ConditionalCommand stopOrIngest(Command halt, Command ingest, boolean intakeVoltageSpike) {
        return stopOrIngest(halt, ingest, intakeVoltageSpike);
    }

    // idk if we need to check for voltagespike while ejecting
    public ConditionalCommand stopOrEject(Command halt, Command eject, boolean intakeVoltageSpike) {
        return stopOrEject(halt, eject, intakeVoltageSpike);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);    
    }
}
