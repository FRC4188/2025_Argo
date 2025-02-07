package frc.robot.commands.autos.autotasks;

import java.util.Comparator;
import java.util.PriorityQueue;

public class ActionManager {
    PriorityQueue<ActionNode> available = new PriorityQueue<ActionNode>(Comparator.comparingDouble(ActionNode::getWeight));
    PriorityQueue<ActionNode> execute = new PriorityQueue<ActionNode>(Comparator.comparingDouble(ActionNode::getWeight));

    public ActionManager(){
        init();
    }

    private void init(){
        //TODO: fill in
    }  

    public void update(){
        for(ActionNode no: available){
            no.update();
        }
        updateNote();
    }

    public void updateDashboard(){
        // Field2d field = new Field2d();
        // SmartDashboard.putData(field);
    }

    private void updateNote(){
        //TODO: research how april tag localization work then apply to note localization
        //e.g. get sensors to constantly check note locations, update weight if needed

    }

    public ActionNode getBestPoint(){
        return execute.poll();
    }
    
}
