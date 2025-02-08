package frc.robot.subsystems.scoring;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Main;
import frc.robot.subsystems.scoring.arm.Arm;
import frc.robot.subsystems.scoring.arm.ArmIO;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.scoring.SuperstructureConfig.*;

import org.littletonrobotics.junction.Logger;

public class SuperVisualizer {
    Mechanism2d mainMech;
    MechanismRoot2d root;
    MechanismLigament2d elevatorLig, armLig, wristLig;
    String key;

    public SuperVisualizer(String logkey){
        mainMech = new Mechanism2d(4, 3);
        root = mainMech.getRoot("superstructure", origin.getX(), origin.getY());

        elevatorLig = root.append(
            new MechanismLigament2d(
                "Ele Carriage", 
                Meters.of(origin.getZ()).magnitude(),
                origin.getRotation().getZ(),
                4,
                new Color8Bit(Color.kBlack)
        ));

        armLig = elevatorLig.append(
            new MechanismLigament2d(
                "arm",
                arm.length(),
                0,
                4,
                new Color8Bit(Color.kAliceBlue))
        );

        wristLig = armLig.append(
            new MechanismLigament2d(
                "wrist", 
                wrist.length(),
                0,
                4,
                new Color8Bit(Color.kAquamarine))
        );

        key = logkey;
    }
public void updateWVoltage(double elevatorInput, double armInput, double wristInput, double elevatorHeight, double armAngle, double wristAngle){
    elevatorHeight = MathUtil.clamp(elevatorHeight, 0, HIGHEST_H - LOWEST_H);
        elevatorHeight = Meters.convertFrom(elevatorHeight, Inches);

        armLig.setAngle(armAngle);
        wristLig.setAngle(wristAngle);
        elevatorLig.setLength(elevatorHeight);
        //Logger.recordOutput("Mechanism2d", mainMech);

        //update pose3d for 3d simulation (ligaments alone are 2d)
        Pose3d carriage = carriageOrigin.transformBy(
            new Transform3d(
                new Translation3d(0, 0, -elevatorHeight),
                new Rotation3d(0, 0, 0)
            )
        );

        //Pose3d armPos = new Pose3d(-0.231965, 0, 0.231965, new Rotation3d(0, Units.degreesToRadians(90), 0));
        Pose3d armPos = armOrigin.transformBy(
            new Transform3d(
                new Translation3d(0, 0, -elevatorHeight),
                new Rotation3d(0, Units.degreesToRadians(armAngle), 0)
            )
        );
        
        Translation2d wrist_rotate = new Translation2d(
            wristOrigin.getTranslation().getX(),
            wristOrigin.getTranslation().getZ()
        ).rotateAround(new Translation2d(
            armOrigin.getX(),
            armOrigin.getZ()
        ), Rotation2d.fromDegrees(armAngle));

        Pose3d wristPos = new Pose3d(
                new Translation3d(wrist_rotate.getX(), wristOrigin.getY(), wrist_rotate.getY()),
                wristOrigin.getRotation()
        ).transformBy(
            new Transform3d(
                new Translation3d(0, 0, -elevatorHeight),
                new Rotation3d(0, Units.degreesToRadians(wristAngle +  armAngle), 0)
            )
        );

        
        Logger.recordOutput("Mechanism3d/" + key, carriage, armPos, wristPos);
   

}

    public void setArmVolts(double volts) {
        double currentAngle = armLig.getAngle();
        armLig.setAngle(currentAngle + volts);
    }


    public void update(double elevatorHeight, double armAngle, double wristAngle){
        elevatorHeight = MathUtil.clamp(elevatorHeight, 0, HIGHEST_H - LOWEST_H);
        elevatorHeight = Meters.convertFrom(elevatorHeight, Inches);


        armLig.setAngle(armAngle);
        wristLig.setAngle(wristAngle);
        elevatorLig.setLength(elevatorHeight);
        //Logger.recordOutput("Mechanism2d", mainMech);

        //update pose3d for 3d simulation (ligaments alone are 2d)
        Pose3d carriage = carriageOrigin.transformBy(
            new Transform3d(
                new Translation3d(0, 0, -elevatorHeight),
                new Rotation3d(0, 0, 0)
            )
        );

        //Pose3d armPos = new Pose3d(-0.231965, 0, 0.231965, new Rotation3d(0, Units.degreesToRadians(90), 0));
        Pose3d armPos = armOrigin.transformBy(
            new Transform3d(
                new Translation3d(0, 0, -elevatorHeight),
                new Rotation3d(0, Units.degreesToRadians(armAngle), 0)
            )
        );
        
        Translation2d wrist_rotate = new Translation2d(
            wristOrigin.getTranslation().getX(),
            wristOrigin.getTranslation().getZ()
        ).rotateAround(new Translation2d(
            armOrigin.getX(),
            armOrigin.getZ()
        ), Rotation2d.fromDegrees(armAngle));

        Pose3d wristPos = new Pose3d(
                new Translation3d(wrist_rotate.getX(), wristOrigin.getY(), wrist_rotate.getY()),
                wristOrigin.getRotation()
        ).transformBy(
            new Transform3d(
                new Translation3d(0, 0, -elevatorHeight),
                new Rotation3d(0, Units.degreesToRadians(wristAngle +  armAngle), 0)
            )
        );

        
        Logger.recordOutput("Mechanism3d/" + key, carriage, armPos, wristPos);
    }
}
