package frc.robot.commands;

import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.FieldConstant;

public final class AutoConfig {
    
    public static Command toBasetoSource(){
        Pose2d currPose = new Pose2d(FieldConstant.Cage.cage_positions[1], new Rotation2d(Rotation2d.fromDegrees(0)));
    }
}
