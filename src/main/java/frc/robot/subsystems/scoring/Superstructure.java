package frc.robot.subsystems.scoring;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.scoring.SuperstructureState.SuperPreset;
import frc.robot.subsystems.scoring.Arm.Arm;
import frc.robot.subsystems.scoring.lift.Elevator;
import frc.robot.subsystems.scoring.wrist.IntakeWrist;

public class Superstructure extends SubsystemBase{
    private final Arm arm;
    private final Elevator elevator;
    private final IntakeWrist wrist;
    
    private SuperPreset state = new SuperPreset.L4_CORAL;
    private SuperPreset next = new SuperPreset();
    private SuperPreset goal = new SuperPreset();
    public Superstructure(Arm arm, Elevator elevator, IntakeWrist wrist){
        this.arm = arm;
        this.elevator = elevator;
        this.wrist = wrist;
    }

   public void setgoal(){}





    @Override
    public void periodic(){

    }

    
    
}