package frc.robot.subsystems.scoring;

public class SuperConstraints {
    /*
     * -angle on wrist = rotate counterclockwise = towards coral intake side
     * 
     * when wrist = -90 height = 0
     *  wrist lowest bound ~ 0 
     *  highest bound = normal
     * when wrist = 90 height = 0
     *  wrist highest bound ~ 10
     *  lowest bound = normal
     */
    
    
    //rough estimate
    int wristHighBound = 100;
    int wristLowBound = -90;
}
