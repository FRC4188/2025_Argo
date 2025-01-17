package frc.robot.subsystems.scoring.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    //no getters for telemetry cuz thats wut the autologged inputs do
    private static Intake instance;
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;

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

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);    }


    
}
