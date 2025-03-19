package frc.robot.subsystems.scoring.superstructure;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.autos.pathgen.PG_math;
import frc.robot.subsystems.scoring.superstructure.SuperConstraints.ElevatorConstraints;
import frc.robot.subsystems.scoring.superstructure.SuperConstraints.WristConstraints;
import frc.robot.util.FieldConstant.Reef;


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
        wrist_angle = angle;
    }


    public void setEleHeight(double height) {
        elevator_height = height;
    }

    public static enum SuperPreset{
        L3_ALGAE(
            new SuperState(
                0.6077, 0.8713)),
 
        L2_ALGAE(
            new SuperState(
                0.7386, 0.4719)),
    
        NET(
            new SuperState(
                0.8, SuperConstraints.ElevatorConstraints.RANGE)),
        PROCESSOR(
            new SuperState(
                0.5,0)),

        ALGAE_GROUND(
             new SuperState(1.2, 0)),
        
        START(
            new SuperState(0, 0)),

        ALGAE_STOW(
            new SuperState(0.5, 0)
        );
        
                
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