package frc.robot.subsystems.scoring.superstructure;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.util.FieldConstant;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.scoring.superstructure.SuperstructureConfig.*;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
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


    public void update(SuperState state){

        armLig.setAngle(state.getArmAngle());
        wristLig.setAngle(state.getWristAngle());
        elevatorLig.setLength(state.getEleHeight());
        //Logger.recordOutput("Mechanism2d", mainMech);

        //update pose3d for 3d simulation (ligaments alone are 2d)
        Pose3d carriage = carriageOrigin.transformBy(
            new Transform3d(
                new Translation3d(0, 0, -state.getEleHeight()),
                new Rotation3d(0, 0, 0)
            )
        );

        Pose3d armPos = armOrigin.transformBy(
            new Transform3d(
                new Translation3d(0, 0, -state.getEleHeight()),
                new Rotation3d(0, state.getArmAngle(), 0)
            )
        );
        
        Translation2d wrist_rotate = new Translation2d(
            wristOrigin.getTranslation().getX(),
            wristOrigin.getTranslation().getZ()
        ).rotateAround(new Translation2d(
            armOrigin.getX(),
            armOrigin.getZ()
        ), Rotation2d.fromRadians(state.getArmAngle()));

        Pose3d wristPos = new Pose3d(
                new Translation3d(wrist_rotate.getX(), wristOrigin.getY(), wrist_rotate.getY()),
                wristOrigin.getRotation()
        ).transformBy(
            new Transform3d(
                new Translation3d(0, 0, -state.getEleHeight()),
                new Rotation3d(0,state.getGlobalAngle(), 0)
            )
        );
        SwerveDriveSimulation driveSim = new SwerveDriveSimulation(Drive.mapleSimConfig, FieldConstant.Reef.CoralGoal.alliance_left);

        Pose3d endEffectorPos = wristPos.transformBy(
            new Transform3d(
                new Translation3d(
                    driveSim.getSimulatedDriveTrainPose().getTranslation().getX(),
                    driveSim.getSimulatedDriveTrainPose().getTranslation().getY(),
                    0),
                new Rotation3d(0, 0, driveSim.getSimulatedDriveTrainPose().getRotation().getRadians())
            ));

        
        Logger.recordOutput("Mechanism3d/" + key, carriage, armPos, wristPos, endEffectorPos);
    }
}
