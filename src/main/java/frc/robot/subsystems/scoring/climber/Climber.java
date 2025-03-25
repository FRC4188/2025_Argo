package frc.robot.subsystems.scoring.climber;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;

public class Climber extends SubsystemBase{
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs;

    public Climber(ClimberIO io) {
        this.io = io;
        inputs = new ClimberIOInputsAutoLogged();
    }

     @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);   
    }

    public void runVolts(double volts) {
        io.runVolts(volts);
    }



    @AutoLogOutput(key = "Climber/Angle Radians")
    public double getAngle(){
        return io.getAngle();
    }

    public boolean atGoal(double target) {
        return Math.abs(getAngle() - target) < ClimberConstants.kTolerance;
    }


}
