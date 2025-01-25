package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{

    //no getters for telemetry cuz thats wut the autologged inputs do
    private static Elevator instance;
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs;
TalonFX RightMotor;
    TalonFX LeftMotor;

    CANcoder RightEncoder;
    CANcoder LeftEncoder;

    double LeftZero = 0;
    double RightZero = 0;

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
    }

    public void stopElevator() {
        // Stop the elevator
        LeftMotor.set(0.0);
            RightMotor.set(0.0);
            
        double leftPosition = LeftEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
        double rightPosition = RightEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    
    
    
        LeftMotor.set(PID.calculate(LeftEncoder.getAbsolutePosition().getValueAsDouble() * 360.0,leftPosition));
        RightMotor.set(PID.calculate(RightEncoder.getAbsolutePosition().getValueAsDouble() * 360.0,rightPosition));
    
    }
    
    public void setElevatorPosition(double position) {
        LeftMotor.set(PID.calculate(LeftEncoder.getAbsolutePosition().getValueAsDouble() * 360.0,position));
        RightMotor.set(PID.calculate(RightEncoder.getAbsolutePosition().getValueAsDouble() * 360.0,position));
    }
    public double getElevatorPosition() {
            
        return (LeftEncoder.getAbsolutePosition().getValueAsDouble() * 360.0)+(RightEncoder.getAbsolutePosition().getValueAsDouble() * 360.0)/2.0;
    }
    
}