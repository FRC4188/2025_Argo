package frc.robot.subsystems.scoring;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.scoring.SuperstructureConfig.*;

import org.littletonrobotics.junction.Logger;

public class SuperVisualizer {
    Mechanism2d main;
    MechanismRoot2d root;
    MechanismLigament2d elevator, arm, wrist;
    String key;

    public SuperVisualizer(String logkey){
        main = new Mechanism2d(4, 3);
        root = main.getRoot("superstructure", origin.getX(), origin.getY());

        elevator = root.append(
            new MechanismLigament2d(
                "Ele Carriage", 
                Meters.of(origin.getZ()).magnitude(),
                origin.getRotation().getZ(),
                4,
                new Color8Bit(Color.kBlack)
        ));

        arm = elevator.append(
            new MechanismLigament2d(
                "arm",
                arm.getLength(),
                0,
                4,
                new Color8Bit(Color.kAliceBlue))
        );

        wrist = arm.append(
            new MechanismLigament2d(
                "wrist", 
                wrist.getLength(),
                0,
                4,
                new Color8Bit(Color.kAquamarine))
        );

        key = logkey;
    }

    public void update(double elevatorHeight, double armAngle, double wristAngle){
        arm.setAngle(armAngle);
        wrist.setAngle(wristAngle);
        elevator.setLength(elevatorHeight);

        //update pose3d for 3d simulation (ligaments alone are 2d)
        Pose3d carriage = 
            new Pose3d(
                origin.getX(),
                origin.getY(),
                elevatorHeight,
                new Rotation3d()
            );
        
        Pose3d armPos = 
            carriage.transformBy(
                new Transform3d(
                    new Translation3d(0.0, 0.0, elevatorHeight),
                    new Rotation3d(-armAngle, 0.0, 0.0)
                )
            );

        Pose3d wristPos = 
            armPos.transformBy(
                new Transform3d(
                    new Translation3d(0.0, arm.getLength(), elevatorHeight),
                    new Rotation3d(-wristAngle, 0.0, 0.0)
                )
            );

        Logger.recordOutput("Mechanism3d/" + key, carriage, armPos, wristPos);
    }
}
