package frc.robot.commands.autos.autotasks;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drive;

public class ActionNode {
    private final Action action;
    private Pose2d location;
    private Boolean isActive = true;
    private double weight = 0;
    private double execRadius;
    private Drive drive;
    // private PropertyChangeSupport support = new PropertyChangeSupport(this);
    

    public ActionNode(Action action, Pose2d location) {
        this.location = location;
        this.action = action;
        this.execRadius = action.getExecRadius();
        // this.support = new PropertyChangeSupport(this);
    }

    public ActionNode(Action action, Drive drive){
        this.action = action;
        this.drive = drive;
        location = drive.getPose();
        this.execRadius = action.getExecRadius();
        // this.support = new PropertyChangeSupport(this);
    }

    //Node is active if robot is in range of the action and the preconditions are met
    private void updateActive(){
        if (checkLocation() && Constants.robot.robotstate == action.getPrecondState()) { //TODO: CHANGE DA CONDITION PLS
            isActive = true;
        }else   
            isActive = false;
    }

    public void update(){
        updateActive();
        /* if sensors detect changes (note and robot detect) = apply changes in weight + exec time */
    }

    public void updateDashboard(String name){
        //setPriority(SmartDashboard.getNumber(name + " priority", 0.0));
    }

    private void checkRisk(){
        double bottomX = 0;
        //TODO: fill in
    }

    public double getDistanceFromCurr(){
        return location.getTranslation().getDistance(drive.getPose().getTranslation());
    }

    public double getExecTime(){
        //TODO: fill in Drive to
        return 0;
    }

    /**
     * Get the weight of importance of going from current (initial) pose to desired pose. (most important = lowest weight)
     * Higher time = higher weight, larger distance = smaller weight (TODO: MAKE THE EQUATION FOR THIS PLS)
     * @param initial pose
     * @param goal pose
     * @return  weight
     */
    public void updateWeight(){
        checkRisk();
        weight = weight - action.getBenefits() + getExecTime() + getDistanceFromCurr(); //TODO: change to better equation, im suck at making shit up
    } 

    public boolean isActive() {
        return isActive;
    }

    public Pose2d getLocation(){
        return location;
    }

    public double getWeight(){
        return weight;
    }

    public Action getAction(){
        return action;
    }

    public void setLocation(Pose2d location){
        if(this.location != location){
            this.location = location;
            updateWeight();
        }
    }

    /**
     * for later use if auto path generation works good.
     * for strat, use this to indicate which note should be prioritized first
     * higher priority value will lead to game element more likely to execute first
     * negative value will result in game element being last in line for execution
     * @param priority value
     */
    private void setPriority(double priority){
        weight -= priority;
    }

    //TODO: depends on the task(command) the execution range of the robot is different (shoot range is bigger than intake range)

    private boolean checkLocation(){
        if(execRadius == 0) return true;
        Pose2d robot = drive.getPose();
        return Math.abs(robot.getX()-location.getX()) <= execRadius
            && Math.abs(robot.getY()-location.getX()) <= execRadius;
    }

     /**
     * @return
     */
    public Command getCommand() {
        switch (action.task) {
            default : return new InstantCommand();
        }
    }
}