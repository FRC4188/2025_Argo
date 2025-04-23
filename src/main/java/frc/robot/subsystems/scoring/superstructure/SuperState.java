package frc.robot.subsystems.scoring.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.scoring.superstructure.SuperConstraints.ElevatorConstraints;
import frc.robot.subsystems.scoring.superstructure.SuperConstraints.WristConstraints;

public class SuperState {

    //radians and meters
    private double wrist_angle = 0;
    private double elevator_height = 0;
    
    public SuperState() {}

    public SuperState(double wrist, double elevator) {
        wrist_angle = MathUtil.clamp(wrist, WristConstraints.LOWEST_A, WristConstraints.HIGHEST_A);
        elevator_height = MathUtil.clamp(elevator, 0, ElevatorConstraints.RANGE);
    }


    public double getWristAngle() {
        return wrist_angle;
    }  

    public double getEleHeight() {
        return elevator_height;
    }

    public void setWristAngle(double angle) {
        wrist_angle = MathUtil.clamp(angle, WristConstraints.LOWEST_A, WristConstraints.HIGHEST_A);
    }


    public void setEleHeight(double height) {
        elevator_height = MathUtil.clamp(height, 0, ElevatorConstraints.RANGE);
    }

    //TODO: right auto + coral + L3
    public static enum SuperPreset{
        L3_ALGAE(
            new SuperState(
                0.7, 0.8713)),
 
        L2_ALGAE(
            new SuperState(
                0.7, 0.4719)),
        
        L1_CORAL(
            new SuperState(
                0, 0.4719 - Units.inchesToMeters(6)
            )
        ),
    
        NET(
            new SuperState(
                1, SuperConstraints.ElevatorConstraints.RANGE)),

        ALGAE_GROUND(
             new SuperState(1.4, 0)),
        
        START(
            new SuperState(0, 0)),

        PROCESSOR(
            new SuperState(0.5, 0)
        ),

        ALGAE_STOW(
            PROCESSOR.getState()
        );
        //Note that processor and stow are the same thing, just separate variables for naming purposes
        
                
        private final SuperState state;

        private SuperPreset(SuperState state){
            this.state = state;
        }
        public SuperState getState(){
            return state;
        }
    }

    public String toString() {
        return String.format("State(Wrist Angle: %.2f, Ele Height: %.2f)", wrist_angle, elevator_height);
    }
}