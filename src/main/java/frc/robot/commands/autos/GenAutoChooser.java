package frc.robot.commands.autos;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;

public class GenAutoChooser {
    private final LoggedDashboardChooser<Pose2d> startPose = new LoggedDashboardChooser<>("Dynamic Auto/Starting Position");
    private final LoggedDashboardChooser<Pose2d> source = new LoggedDashboardChooser<>("Dynamic Auto/Main source");
    private final LoggedDashboardChooser<Pose2d> firstGoal = new LoggedDashboardChooser<>("Dynamic Auto/1st Goal");
    private final LoggedDashboardChooser<Pose2d> secndGoal = new LoggedDashboardChooser<>("Dynamic Auto/2nd Goal");
    private final LoggedDashboardChooser<Pose2d> thirdGoal = new LoggedDashboardChooser<>("Dynamic Auto/3rd Goal");
    private final LoggedDashboardChooser<Pose2d> fourthGoal = new LoggedDashboardChooser<>("Dynamic Auto/4th Goal");

    //DS pov
    public static enum StartLoc{
        MID("Middle", new Pose2d()),
        LEFT("Left", new Pose2d()),
        RIGHT("Right", new Pose2d());

        String key;
        Pose2d pose;
        private StartLoc(String key, Pose2d pose){
            this.key = key;
            this.pose = pose;
        }
    }
    public void init(){
        startPose.addOption(StartLoc.MID.key, StartLoc.MID.pose);
        startPose.addOption(StartLoc.LEFT.key, StartLoc.LEFT.pose);
        startPose.addOption(StartLoc.RIGHT.key, StartLoc.RIGHT.pose);
    }
}
