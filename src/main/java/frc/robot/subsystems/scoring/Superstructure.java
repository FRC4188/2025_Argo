package frc.robot.subsystems.scoring;
import java.time.LocalDate;

import org.opencv.video.KalmanFilter;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.scoring.SuperState.*;
import frc.robot.subsystems.scoring.arm.Arm;
import frc.robot.subsystems.scoring.elevator.Elevator;
import frc.robot.subsystems.scoring.wrist.Wrist;

public class Superstructure extends SubsystemBase{
    private final Arm arm;
    private final Elevator elevator;
    private final Wrist wrist;

    ArmFF ff;
    ArmKinematics kinematics;

    private final Constraints constraints = new Constraints(960.0, 720.0);

    private ProfiledPIDController armPID = 
        new ProfiledPIDController(
            0.1, 0.0, 0.0, 
            constraints);

    private ProfiledPIDController wristPID = 
        new ProfiledPIDController(
            0.1, 0.0, 0.0, 
            constraints);

    private ProfiledPIDController elePID = 
        new ProfiledPIDController(
            0.1, 0.0, 0.0, 
            constraints);
        
    private SuperPreset target;
    
    public Superstructure(Arm arm, Elevator elevator, Wrist wrist){
        target = SuperPreset.START;
        this.arm = arm;
        this.elevator = elevator;
        this.wrist = wrist;

        ff = new ArmFF();
        kinematics = new ArmKinematics();
    }

    public void setgoal(SuperPreset goal){
        target = goal;
    }

    public void setSystem(double wristAngle, double armAngle, double heightInch){
        elevator.runVolts(
            elePID.calculate(elevator.getHeight(), heightInch));

        arm.setVolt(
                armPID.calculate(arm.getAngle(), armAngle));
        wrist.runVolts(
                wristPID.calculate(wrist.getAngle(), wristAngle));
    }

    @Override
    public void periodic(){
        var state = target.getState();
        arm.setVolt(
            armPID.calculate(arm.getAngle(), state.getArmAngle())
            + ff.getArmVoltFF(VecBuilder.fill(state.endEffectorPos().getX(), state.endEffectorPos().getZ()))
        );
        // didnt know i had to finish this class mb ig
        elevator.runVolts(
            elePID.calculate(elevator.getHeight(), state.getHeightInch())
        );
        wrist.setAngle(
            wristPID.calculate(wrist.getAngle(), state.getWristAngle() + target.getWristOffset())
            + ff.getWristVoltFF(VecBuilder.fill(state.endEffectorPos().getX(), state.endEffectorPos().getZ()))
            // Hopefully this is the right FF arguemnts for the wrist
        );
    }

    
    
}//change