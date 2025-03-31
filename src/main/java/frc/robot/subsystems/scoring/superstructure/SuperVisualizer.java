package frc.robot.subsystems.scoring.superstructure;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import static frc.robot.subsystems.scoring.superstructure.SuperstructureConfig.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class SuperVisualizer {
    private String key;

    private DoubleSupplier elepos;
    private DoubleSupplier wristpos;
    private DoubleSupplier climberpos;

    public SuperVisualizer(String logkey, DoubleSupplier ele, DoubleSupplier wrist, DoubleSupplier climber){
        elepos = ele;
        wristpos = wrist;
        climberpos = climber;

        key = logkey;
    }

    public void update(){
        Pose3d carriage = origin.transformBy(
            new Transform3d(
                new Translation3d(0, 0, elepos.getAsDouble()),
                new Rotation3d(0, 0, 0)
            )
        );

        Pose3d wristPos = wristAxis.transformBy(
            new Transform3d(
                new Translation3d(0, 0, elepos.getAsDouble()),
                new Rotation3d(0, wristpos.getAsDouble(), 0)
            )
        );

        Pose3d climberPos = climberAxis.transformBy(
            new Transform3d(
                new Translation3d(),
                new Rotation3d(0, climberpos.getAsDouble(), 0)
            )
        );
                
        Logger.recordOutput("Mechanism3d/" + key, carriage, wristPos, climberPos);
    }
}