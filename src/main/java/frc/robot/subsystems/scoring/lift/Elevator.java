package frc.robot.subsystems.scoring.lift;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.scoring.SuperstructureConfig;

public class Elevator extends SubsystemBase{

    //no getters for telemetry cuz thats wut the autologged inputs do
    private static Elevator instance;
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs;

    public PIDController elevatorPID = new PIDController(0.1, 0.0, 0.0);

    private double targetHeight = 0;
    private double elevatorHeight = 0;

    ElevatorFeedforward PID;
    double speedAfterPID;
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

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);   
        
        targetHeight = MathUtil.clamp(targetHeight, SuperstructureConfig.LOWEST_H, SuperstructureConfig.HIGHEST_H);
        elevatorHeight = inputs.posRads; // TODO: Change PosRads to height
        runPosition(targetHeight);
    }
    public Command runVolts(double volts){
        return run(()-> io.runVolts(volts));
    }
    public Command runPosition(double height){
        return run(()-> io.runPosition(height));
    }

    public double getElevatorHeight(){
        return elevatorHeight;
    }

    public Command setHeight(double height){
        return run(()->{
            targetHeight = height;
        });
    }

    
}