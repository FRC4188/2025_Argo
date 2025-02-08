package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.scoring.SuperstructureState.SuperState;

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
    // int elevatorDownArmLowBound = -90;
    // int elevatorDownArmHighBound = 90;
    // int elevatorUpArmLowBound = -175;
    // int elevatorUpArmHighBound = 175;
    public final static double LOWEST_H = Inches.of(9.13250).magnitude();//0.228362383 meters
    public final static double HIGHEST_H = Inches.of(9.13250 + 72.0).magnitude();//2.057162383 meters
    public final double LOWEST_H_A_180 = Inches.of(30).magnitude();


    //make sure arm is always [-80, 90] at height 0, with extension to 180 at min height of lowestha180 
    //make sure arm doesn't rotate continuously beyond 360 or below 0
    //TODO: make so arm + wrist cannot go past elevator when has algae (EXTREMELY IMPORTANT)

    public SuperState getOptimized(SuperState state) {
        double safeInterpo = (state.getHeightInch() / LOWEST_H_A_180) + 0.05; //TODO: tune 0.05 for a safety margin
        // if (state.getHeightInch() >= 30) {
        //     int armLowBound = -150;
        //     int armHighBound = 150;
        //     MathUtil.clamp(state.getArmAngle(), armLowBound, armHighBound);
        // } else {
        //     int armLowBound = 0;
        //     int armHighBound = 90;
        //     MathUtil.clamp(state.getArmAngle(), armLowBound, armHighBound);
        // }
        
        return state;
    
    }
}
