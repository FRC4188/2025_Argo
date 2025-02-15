package frc.robot.commands.drive;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.generated.TunerConstants;
import frc.robot.subsystems.vision.Limelight;

public class TagTracking extends Command{
    double tagAngle = 0.0;
    Drive drive;
    PIDController rotPID = new PIDController(0.0, 0.0, 0.0);
    String llName;
    @Override
    public void initialize(){

    }
    public TagTracking(Drive drive, String llName){
        this.llName = llName;
        this.drive = drive;
        
    }

    @Override
    public void execute(){
        tagAngle = Limelight.getTX(llName);
        drive.drive(0.0, 0.0, rotPID.calculate(tagAngle, 0.0), isScheduled(), isFinished());
    }

    @Override
    public void end(boolean interuppted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
