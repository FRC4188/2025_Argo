package frc.robot.subsystems.scoring;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.autos.pathgen.PG_math;

import static frc.robot.subsystems.scoring.SuperstructureConfig.*;

import java.util.ArrayList;

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
    
    public static Translation2d forward(Vector<N2> angles) {
        return new Translation2d(
            origin.getX()
                + arm.length() * Math.cos(angles.get(0, 0))
                + wrist.length() * Math.cos(angles.get(0, 0) + angles.get(1, 0)),
            origin.getY()
                + arm.length() * Math.sin(angles.get(0, 0))
                + wrist.length() * Math.sin(angles.get(0, 0) + angles.get(1, 0)));
    }

    public static Translation3d fromPose(Translation2d t, boolean isCoral) {
        //get arm angle from optimal wrist angle
        double a_length = Units.inchesToMeters(16.1378264837);
        double w_length = Units.inchesToMeters(15.9504541222); //algae

        double x = MathUtil.clamp(t.getX(), -w_length-a_length, w_length + a_length);
        double z = MathUtil.clamp(t.getY(), 0, SuperConstraints.ElevatorConstraints.HIGHEST_H + a_length);

        ArrayList<Translation2d> possible_vals = new ArrayList<Translation2d>();

        for (double a = -Math.PI; a < Math.PI; a += 0.2) {
            double val1 = Math.PI - Math.asin((x - a_length * Math.sin(a))/w_length) - a;
            double val2 = Math.asin((x - a_length * Math.sin(a))/w_length) - a;

            if (!(Double.isNaN(val1) && Double.isNaN(val2))) {

                double w_val = 0;
                if (Double.isNaN(val1)) {
                    w_val = PG_math.modulate(Rotation2d.fromRadians(val2)).getRadians();
                } else if (Double.isNaN(val2)) {
                    w_val = PG_math.modulate(Rotation2d.fromRadians(val1)).getRadians();
                } else {
                    val1 = PG_math.modulate(Rotation2d.fromRadians(val1)).getRadians();
                    val2 = PG_math.modulate(Rotation2d.fromRadians(val2)).getRadians();
                    w_val = (Math.abs(val2) > Math.abs(val1))?val1:val2;
                }

                if (w_val <= 5/9.0 * Math.PI && w_val >= -5/9.0 * Math.PI) {

                    possible_vals.add(new Translation2d(w_val, a));
                }
            }
        }

        possible_vals.sort((Translation2d a1, Translation2d a2) -> 
            (Double.compare(
                Math.hypot(a1.getX(), a1.getY()), 
                Math.hypot(a2.getX(), a2.getY())))
        );

        if (possible_vals.isEmpty()) return new Translation3d();

        double result_w = possible_vals.get(0).getX();
        double result_a =  possible_vals.get(0).getY();
        double result_e = MathUtil.clamp(z - SuperConstraints.ElevatorConstraints.LOWEST_H
             - a_length * Math.cos(result_a)
              - w_length * Math.cos(result_w + result_a),
              0,
              SuperConstraints.ElevatorConstraints.RANGE);

        return new Translation3d(result_w, result_a, result_e);
    } 

    public static Translation2d fromValues(double w_angle, double a_angle, double e_height, boolean isCoral) {
        double a_length = Units.inchesToMeters(16.1378264837);
        double w_length = Units.inchesToMeters(15.9504541222); //algae

        double x = a_length * Math.sin(a_angle) + w_length * Math.sin(w_angle + a_angle);
        double z = e_height + SuperConstraints.ElevatorConstraints.LOWEST_H + a_length * Math.cos(a_angle) + w_length * Math.cos(w_angle + a_angle);

        return new Translation2d(x, z);
    }
    
}