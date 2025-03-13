package frc.robot.subsystems.scoring.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.MotorSafety;
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

    public static Intake getInstance(IntakeIO io){
        if (instance == null){
            instance = new Intake(io);
        }
        return instance;
    }

    public Intake(IntakeIO io){
        this.io = io;
        inputs = new IntakeIOInputsAutoLogged();
    }

    public void isSafetyOn(boolean isSafe) {
        io.isSafetyOn(isSafe);
    }

    public void runVolts(double volts){
        io.runVolts(volts);
    }

    public void stop(){
        io.stop();
    }

    public Command ingest() {
        return Commands.run(
            () -> {
                runVolts(3);
            });
    }

    public Command eject() {
        return Commands.run(
            () -> {
                runVolts(-3);
            });
    }

    public Command halt() {
        return Commands.run(
            () -> {
                stop();
            });
    }

    public ConditionalCommand stopOrIngest(Command halt, Command ingest, boolean isSafe) {
        return stopOrIngest(halt, ingest, isSafe);
    }

    // idk if we need to check for voltagespike while ejecting
    public ConditionalCommand stopOrEject(Command halt, Command eject, boolean isSafe) {
        return stopOrEject(halt, eject, isSafe);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);    
    }
}
