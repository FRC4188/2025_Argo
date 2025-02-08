package frc.robot.subsystems.scoring;

import edu.wpi.first.math.MathUtil;
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


    public void update(double elevatorHeight, double armAngle, double wristAngle){
        elevatorHeight = MathUtil.clamp(elevatorHeight, 0, SuperConstraints.HIGHEST_H - SuperConstraints.LOWEST_H);
        elevatorHeight = Meters.convertFrom(elevatorHeight, Inches);
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
        //wrist's gear to algae specific gummy wheel is 14.26 inches

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
