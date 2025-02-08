package frc.robot.subsystems.scoring;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.subsystems.scoring.SuperstructureConfig.*;

//https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
public class ArmKinematics {
    //0,0 = x; 0,1 = y
    public static Vector<N2> apply(Pose2d endPos){
        double angle1 = -Math.cos(
            (Math.pow(endPos.getX(), 2) 
            + Math.pow(endPos.getY(), 2) 
            - Math.pow(arm.length(), 2) 
            - Math.pow(wrist.length(), 2)) / (2 * arm.length() * wrist.length()));

        double angle2 = Math.atan(endPos.getY() / endPos.getX())

             + Math.atan((wrist.length() * Math.sin(angle1))
             /(arm.length() + wrist.length() * Math.cos(angle1)));

        return VecBuilder.fill(angle1, angle2);
    }
    
    public Translation2d forward(Vector<N2> angles) {
        return new Translation2d(
            origin.getX()
                + arm.length() * Math.cos(angles.get(0, 0))
                + wrist.length() * Math.cos(angles.get(0, 0) + angles.get(1, 0)),
            origin.getY()
                + arm.length() * Math.sin(angles.get(0, 0))
                + wrist.length() * Math.sin(angles.get(0, 0) + angles.get(1, 0)));
      }
    
}
