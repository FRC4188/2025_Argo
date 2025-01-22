package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{

    //no getters for telemetry cuz thats wut the autologged inputs do
    private static Elevator instance;
    private final ElevatorIO io;
    private final Elevator inputs;

    public static Elevator getInstance(ElevatorIO io){
        if(instance == null){
            instance = new Elevator(io);
        }
        return instance;
    }

    private Elevator(ElevatorIO io){
        this.io = io;
        //Ansh said Ly said this will be generated.
        inputs = new ElevatorIOInputsAutoLogged();
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
        Logger.processInputs("Elevator", inputs);    }


    
}