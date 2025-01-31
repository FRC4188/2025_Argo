package frc.robot.subsystems.scoring;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class SuperVisualizer {
    Mechanism2d main;
    MechanismRoot2d root;
    MechanismLigament2d elevator, arm, wrist;

    public SuperVisualizer(String logName){
        main = new Mechanism2d(4, 3);
        
    }
}
