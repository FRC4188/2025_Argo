package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scoring.intake.Intake;

public class ScoreNet extends Command {

    private Intake intake;

    public ScoreNet(Intake intake) {
        this.intake = intake;
    }

    public void execute() {
        if (intake.isStalled()) {intake.eject();} else {intake.ingest();}
    }

    public void end(boolean interrupted) {
        intake.stop();
    }
    
}
