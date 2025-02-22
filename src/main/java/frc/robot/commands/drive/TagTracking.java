package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.vision.VisConstants;

public class TagTracking extends Command{
    double tagAngle = 0.0;
    Drive drive;
    PIDController rotPID = Constants.robot.TURN_PID;
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
        // tagAngle = Limelight.getTX(llName);
        tagAngle = VisConstants.AprilTagPose.tag20.getRotation().getZ();
        drive.drive(0.0, 0.0, rotPID.calculate(drive.getPose().getRotation().getRadians(), tagAngle), isScheduled(), isFinished());
    }

    @Override
    public void end(boolean interuppted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
