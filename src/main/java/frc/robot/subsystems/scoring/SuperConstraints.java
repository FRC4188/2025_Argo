package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class SuperConstraints {
    public class ArmConstraints {
        public final static double LOWEST_A = Units.degreesToRadians(-180);
        public final static double HIGHEST_A = Units.degreesToRadians(180);
    }

    public class WristConstraints{
        public final static double LOWEST_A = Units.degreesToRadians(-100);
        public final static double HIGHEST_A = Units.degreesToRadians(100);
    }

    public class ElevatorConstraints{
        public final static double LOWEST_H = Units.inchesToMeters(9.13250);
        public final static double HIGHEST_H = Units.inchesToMeters(9.13250 + 72);
        public final static double RANGE = HIGHEST_H - LOWEST_H;
    }

    public SuperState getOptimized(SuperState state) {
        double elevatorHeight = state.getHeight();
        double armAngle = state.getArmAngle();
        double wristAngle = state.getWristAngle();

        elevatorHeight = MathUtil.clamp(elevatorHeight, 0, ElevatorConstraints.RANGE);
        armAngle = MathUtil.clamp(armAngle, -Math.PI, Math.PI);
        wristAngle = MathUtil.clamp(armAngle, -5/9.0 * Math.PI, 5/9.0 * Math.PI);

        return new SuperState(elevatorHeight, armAngle, wristAngle, true);
    }
}
