package frc.robot.commands.drive;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.inputs.CSP_Controller;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.generated.TunerConstants;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.VisConstants;
import frc.robot.util.FieldConstant;

public class TrackingDrive extends Command{
    double angle = 0.0;
    Drive drive;
    PIDController rotPID = new PIDController( //could be different values for tracking maybe
        TunerConstants.steerGains.kP,
        TunerConstants.steerGains.kI,
        TunerConstants.steerGains.kD);
    boolean isMoving;
    String llName;
    CSP_Controller controller;
    @Override
    public void initialize(){
        rotPID.setTolerance(2.0);
    }

    public TrackingDrive(Drive drive, String llName, CSP_Controller controller){
        this.llName = llName;
        this.drive = drive;
        this.controller = controller;
    }

    @Override
    public void execute(){
        angle = Limelight.frontTX();

        // the isMoving case definitely needs to be changed, look at techno 2024 alignment for reference
        if (isMoving){
            DriveCommands.TeleDrive(
                drive, 
                ()-> controller.getLeftY(), 
                ()-> 0.0, 
                ()-> rotPID.calculate(drive.getPose().getRotation().getDegrees(), angle));
        }
        else{
            DriveCommands.TeleDrive(
                drive, 
                ()-> 0.0, 
                ()-> 0.0, 
                ()-> rotPID.calculate(drive.getPose().getRotation().getRadians(), angle));
        }
    }

    @Override
    public void end(boolean interuppted){
        drive.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}