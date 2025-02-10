
package frc.robot.subsystems.scoring.elevator;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.subsystems.scoring.SuperConstraints;
import frc.robot.subsystems.scoring.SuperstructureConfig;

import static frc.robot.Constants.ElevatorConstants.*;

//akhil is mid - ansh, akhil is goofy goober - ansh
public class Elevator extends SubsystemBase{

    //no getters for telemetry cuz thats wut the autologged inputs do
    private static Elevator instance;
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs;

    private double targetHeight;
    private double currentHeight;

    ElevatorFeedforward ff;
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
        ff = new ElevatorFeedforward(
            kMotorConfig.Slot0.kS, 
            kMotorConfig.Slot0.kS, 
            kMotorConfig.Slot0.kS); //TODO: get values from sysid
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);   
        currentHeight = (inputs.posRads - kZero) / 6 * kDrumeRadius * 2; //TODO: test da math

    }
    public double getHeight(){
        return currentHeight;
    }
    // public double convertToPos(double inches){
    //     return inches*Constants.ElevatorConstants.ticksToInches;
    // }
    public Command runVolts(double volts){
        return run(()-> io.runVolts(volts));
    }
    public Command runPosition(double height){
        return run(()-> io.runPosition(height / 3 / kDrumeRadius * 6 - kZero, ff.calculate(960.0))); //TODO: test da math
    }

    public Command runSysId(){
        SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(4).per(Seconds),
                Volts.of(8),
                Seconds.of(6)
            ),new SysIdRoutine.Mechanism(
                voltage -> runVolts(voltage.magnitude()),
                null,
                this));
        
        return Commands.sequence(
            routine.quasistatic(Direction.kForward)
                .until(() -> getHeight() >= SuperConstraints.ElevatorConstraints.RANGE),
            routine.quasistatic(Direction.kReverse)
                .until(() -> getHeight() <= 0.0),
                
            routine.dynamic(Direction.kForward)
                .until(() -> getHeight() >= SuperConstraints.ElevatorConstraints.RANGE),
            routine.dynamic(Direction.kReverse)
                .until(() -> getHeight() <= 0.0)
        );
    }
}
