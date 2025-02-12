package frc.robot.subsystems.scoring;
import java.time.LocalDate;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.scoring.SuperstructureState.SuperPreset;
import frc.robot.subsystems.scoring.arm.Arm;
import frc.robot.subsystems.scoring.lift.Elevator;
import frc.robot.subsystems.scoring.wrist.IntakeWrist;

public class Superstructure extends SubsystemBase{
    private final Arm arm;
    private final Elevator elevator;
    private final IntakeWrist wrist;

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
        
    private SuperPreset state =  SuperPreset.L4_CORAL;
    public Superstructure(Arm arm, Elevator elevator, IntakeWrist wrist){
        this.arm = arm;
        this.elevator = elevator;
        this.wrist = wrist;
    }

   public void setgoal(SuperPreset goal){
        var state = goal.getState();

        arm.setVolt(
            armPID.calculate(arm.getAngle(), state.getArmAngle())
            + ArmFF.getArmVoltFF(VecBuilder.fill(arm.getAngle(), wrist.getMotorAngle()))
        );
   }





    @Override
    public void periodic(){

    }

    
    
}//change