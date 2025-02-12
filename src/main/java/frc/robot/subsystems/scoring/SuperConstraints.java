package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

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
    public final double LOWEST_H_A_180 = Inches.of(30).magnitude();


    //make sure arm is always [-80, 90] at height 0, with extension to 180 at min height of lowestha180 
    //make sure arm doesn't rotate continuously beyond 360 or below 0
    //TODO: make so arm + wrist cannot go past elevator when has algae (EXTREMELY IMPORTANT)

    public class ArmConstraints {
        public final static double GLOBAL_LOWEST_A = -179;
        public final static double GLOBAL_HIGHEST_A = 179;
        public static double LOWEST_A = -179;
        public static double HIGHEST_A = 179;

        //TODO: add math that scale range based on height
        public void setArmConstraints(double lowestA, double highestA){
            LOWEST_A = lowestA;
            HIGHEST_A = highestA;
        }
    }

    public class WristConstraints{
        public final static double GLOBAL_LOWEST_A = -100;
        public final static double GLOBAL_HIGHEST_A = 100;
        public static double LOWEST_A = -100;
        public static double HIGHEST_A = 100;

        public void setArmConstraints(double lowestA, double highestA){
            LOWEST_A = lowestA;
            HIGHEST_A = highestA;
        }
    }

    public class ElevatorConstraints{
        public final static double LOWEST_H = Inches.of(9.13250).magnitude();//0.228362383 meters
        public final static double HIGHEST_H = Inches.of(9.13250 + 72.0).magnitude();//2.057162383 meters
        public final static double RANGE = HIGHEST_H - LOWEST_H;
    }


    public SuperState getOptimized(SuperState state) {
        // if (state.getHeightInch() >= 30) {
        //     int armLowBound = -150;
        //     int armHighBound = 150;
        //     MathUtil.clamp(state.getArmAngle(), armLowBound, armHighBound);
        // } else {
        //     int armLowBound = 0;
        //     int armHighBound = 90;
        //     MathUtil.clamp(state.getArmAngle(), armLowBound, armHighBound);
        // }
        double elevatorHeight = state.getHeightInch();
        double armAngle = state.getArmAngle();
        double wristAngle = state.getWristAngle();

        elevatorHeight = MathUtil.clamp(elevatorHeight, 0, SuperConstraints.ElevatorConstraints.HIGHEST_H - SuperConstraints.ElevatorConstraints.LOWEST_H);
        elevatorHeight = Meters.convertFrom(elevatorHeight,Inches);
        // all of these if and if else statements are just to implement the constraints for wrist and arm 
        // directly/indirectly based on elevator height
        if (elevatorHeight >= Meters.convertFrom(25, Inches)) {
            armAngle = MathUtil.clamp(armAngle, -170, 170);
        } else if (Meters.convertFrom(5, Inches) <= elevatorHeight && elevatorHeight < Meters.convertFrom(10, Inches)){
            armAngle = MathUtil.clamp(armAngle, -115, 115);
        } else if (Meters.convertFrom(10, Inches) <= elevatorHeight && elevatorHeight < Meters.convertFrom(15, Inches)){
            armAngle = MathUtil.clamp(armAngle, -125, 125);
        } else if (Meters.convertFrom(15, Inches) <= elevatorHeight && elevatorHeight < Meters.convertFrom(20, Inches)){
            armAngle = MathUtil.clamp(armAngle, -140, 140);
        } else if (Meters.convertFrom(20, Inches) <= elevatorHeight && elevatorHeight < Meters.convertFrom(25, Inches)){
            armAngle = MathUtil.clamp(armAngle, -155, 155);
        } else {
            armAngle = MathUtil.clamp(armAngle, -90, 90);
        }


        if (-40 <= armAngle && armAngle <= 40) {
            wristAngle = MathUtil.clamp(wristAngle, -100, 100);
        } else if (-90 <= armAngle && armAngle < -40) {
            wristAngle = MathUtil.clamp(wristAngle, 0, 40);
        } else if (-115 <= armAngle && armAngle < -90) {
            wristAngle = MathUtil.clamp (wristAngle, 40, 100);
        } else if (40 < armAngle && armAngle <= 90) {
            wristAngle = MathUtil.clamp(wristAngle, -40, 0);
        } else if (90 < armAngle && armAngle <= 115) {
            wristAngle = MathUtil.clamp(wristAngle, -100, -40);
        } else if (-130 <= armAngle && armAngle <-115) {
            wristAngle = MathUtil.clamp(wristAngle, 55, 100);
        } else if (115 < armAngle && armAngle <= 130) {
            wristAngle = MathUtil.clamp(wristAngle, -100, -50);
        } else if (-145 <= armAngle && armAngle < -125) {
            wristAngle = MathUtil.clamp(wristAngle, 70, 100);
        } else if (125 < armAngle && armAngle <= 140) {
            wristAngle = MathUtil.clamp(wristAngle, -100, -55);
        } else if (140 < armAngle && armAngle <= 155) {
            wristAngle = MathUtil.clamp(wristAngle, -100, -65);
        } else if (-155 <= armAngle && armAngle < -145) {
            wristAngle = MathUtil.clamp(wristAngle, 65, 100);
        } else if (-170 <= armAngle && armAngle < -155) {
            wristAngle = MathUtil.clamp(wristAngle, -100, 100);
        } else if (155 < armAngle && armAngle <= 170) {
            wristAngle = MathUtil.clamp(wristAngle, -100, 100);
        }

        Translation2d xy = ArmKinematics.forward(VecBuilder.fill(armAngle, wristAngle));

        return new SuperState(new Translation3d(xy.getX(), xy.getY(), elevatorHeight));
    
    }
}
