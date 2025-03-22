
package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceFlip {

    public static double flipX(double x){
        if (canFlip()) {
            return FieldConstant.field_length - x;
        } else {
            return x;
        }
    }

    public static double flipY (double y){
        if (canFlip()) {
            return FieldConstant.field_width - y;
        } else {
            return y;
        }
    }

    public static Pose2d flipDS(Pose2d pos){
        if (canFlip()) {
            return new Pose2d(flipX(pos.getX()), flipY(pos.getY()), apply(pos.getRotation()));
        } else {
            return pos;
        }
    }

    /** Flips a rotation based on the current alliance color. */
    public static Rotation2d apply(Rotation2d rotation) {
        if (canFlip()) {
            return new Rotation2d(-rotation.getCos(), -rotation.getSin());
        } else {
            return rotation;
        }
    }

    public static boolean canFlip(){
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }
}
