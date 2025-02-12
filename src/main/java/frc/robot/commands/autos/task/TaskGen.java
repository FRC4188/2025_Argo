package frc.robot.commands.autos.task;

import java.util.Set;

import frc.robot.subsystems.scoring.SuperState.SuperPreset;

public class TaskGen {
    
    /*
     * coral l1-4 = 3, 4, 6, 7
     * algae processor = 6, net =  4
     * 
     * coop = 2 alg in proc
     * coral rp = 5 each level, 5 on 3 if coop
     * auto rp = 1 coral
     * 
     * unloaded -> game ele = loaded -> goals -> unloaded
     * 
     * vertices = state/goal 
     * edge = go to pos super + drive
     * 
     * each task = a cluster of state that can branch to 
     * therefore its basically a group to group + then pick the best one out of the group base on past + future goal
     * GOAL: TIME + POINT OPTIMAL 
     * 
     */

     //planning, not sure if use
    Set
     
     public enum Task{
        CORAL_IN,
        ALGAE_IN,
        CORAL_OUT,
        ALGAE_OUT,
        PROCESSOR,
        NET
     }
}
