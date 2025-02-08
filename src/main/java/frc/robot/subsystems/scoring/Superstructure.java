package frc.robot.subsystems.scoring;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.scoring.SuperStructureState;
import frc.robot.subsystems.scoring.Arm.Arm;
import frc.robot.subsystems.scoring.lift.Elevator;
import frc.robot.subsystems.scoring.wrist.IntakeWrist;

public class Superstructure extends SubsystemBase{
    private final Arm arm;
    private final Elevator elevator;
    private final IntakeWrist wrist;
    
    private SuperStructureState state = new SuperStructureState();
    private SuperStructureState next = new SuperStructureState();
    private SuperStructureState goal = new SuperStructureState();
    public Superstructure(Arm arm, Elevator elevator, IntakeWrist wrist){
        this.arm = arm;
        this.elevator = elevator;
        this.wrist = wrist;
    }


    @Override
    public void periodic(){

    }

    
    
}