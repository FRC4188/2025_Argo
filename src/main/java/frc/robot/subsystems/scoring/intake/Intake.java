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
import frc.robot.Constants.robot;
import frc.robot.Constants.robot.*;

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

    private Intake(IntakeIO io){
        this.io = io;
        inputs = new IntakeIOInputsAutoLogged();
    }


    public static enum GamePieceType {
        CORAL,
        ALGAE
    }

    public boolean isSafetyOn() {
        return io.isSafetyOn();
    }

    public void runVolts(double volts){
        io.runVolts(volts);
    }


    //TODO: fix inverted for coral/algae ingest/eject (don't know which is inverted and which one isn't)
    public Command ingest() {
        return Commands.run(
            () -> {
                switch (robot.intakeState) {
                    case CORAL:
                        io.invertMotor(false);
                        break;
                    case ALGAE:
                        io.invertMotor(true);
                        break;
                    default:
                        break;
                }
                runVolts(1);
            }).until(()-> isSafetyOn());
    }

    public Command eject() {
        return Commands.run(
            () -> {
                GamePieceType type = GamePieceType.CORAL;
                switch (type) {
                    case CORAL:
                        io.invertMotor(true);
                        break;
                    case ALGAE:
                        io.invertMotor(false);
                        break;
                }
                runVolts(1);
            }).until(() -> isSafetyOn());
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
