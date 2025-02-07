package frc.robot.commands.autos.autotasks;

import org.ejml.dense.row.factory.DecompositionFactory_FDRM;

import frc.robot.Constants;
import frc.robot.Constants.robot.STATE;

import static frc.robot.Constants.*;

public class Action {
    // public boolean isAvailable();
    
    // public void setSetpoint(String key, DoubleSupplier value);

    // public Translation2d getGoalPoint();

    // public Map<String, DoubleSupplier> getRequirement();

    // public Map<String, DoubleSupplier> getSetpoint();
    public Task task;
    private ActionNode no;


    public Action(Task task) {
        this.task = task;
        this.no = no;
    }


    public class ActionBenefit{
        public static final int L4 = 0;
        public static final int L3 = 0;
        public static final int L2 = 0;
        public static final int L1 = 0;
        public static final int REMOVE_ALGAE = 0;
        public static final int SOURCE_CORAL = 0;
        public static final int PROCESSOR = 0;
    }

    public static enum Task{
        TRAVEL, CORAL_IN, CORAL_OUT, ALGAE_IN, ALGAE_OUT
    }


    public STATE getPrecondState(){
        switch (task) {
            case CORAL_IN: 
            case ALGAE_IN: 
                return STATE.EMPTY;
            case CORAL_OUT:
                return STATE.CORAL;
            case ALGAE_OUT:
                return STATE.ALGAE;
            default:
                return Constants.robot.robotstate;
        }
    }

    public double getBenefits(){
        switch(task){
            case TRAVEL:
                return 1;
            case CORAL_IN:
                return 5;
            case CORAL_OUT:
                return 0;
            case ALGAE_IN:
                return 0;
            case ALGAE_OUT:
                return 0;
            default:
                return 0;
        }
    }

    public double getExecRadius(){
        switch(task){
            case TRAVEL:
                return 1;
            case CORAL_IN:
                return 5;
            case CORAL_OUT:
                return 0;
            case ALGAE_IN:
                return 0;
            case ALGAE_OUT:
                return 0;
            default:
                return 0;
        }
    }
}