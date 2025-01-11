package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class VisionIOInputsAutoLogged extends VisionIOInputs implements LoggableInputs{

    @Override
    public void toLog(LogTable table) {
        table.put("connected", connected);
        table.put("target",latestTargetObservation);
        table.put("pose", poseObservations);
        table.put("detected tag id", tagIds);
    }

    @Override
    public void fromLog(LogTable table) {
        connected = table.get("connected", connected);
        latestTargetObservation = table.get("target", latestTargetObservation);
        poseObservations = table.get("pose", poseObservations);
        tagIds = table.get("detected tag id", tagIds);
    }
    
}
