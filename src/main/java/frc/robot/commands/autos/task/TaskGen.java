package frc.robot.commands.autos.task;

import java.util.LinkedList;
import java.util.function.BooleanSupplier;

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
    static class Graph{
        int verticesNo;
        LinkedList<Vertex>[] graph;

        @SuppressWarnings("unchecked")
        public Graph(int verticesNo){
            this.verticesNo = verticesNo;
            graph = new LinkedList[verticesNo];
            for (int i = 0; i < verticesNo ; i++) {
                graph[i] = new LinkedList<>();
            }
        }
        
        public void addEdge(Vertex from, Vertex to){
            graph[from.task].add(to);
        }
    }

    public record Vertex(int task, BooleanSupplier precond, double benefits){
        public Vertex(int task){
            this(task, () -> true, 0);
        }
    };

    Graph graph = new Graph(0);
    
    
     
     public class Task{
        static int CORAL_IN ;
        static int ALGAE_IN;
        static int CORAL_OUT;
        static int ALGAE_OUT;
        static int PROCESSOR;
        static int NET;
     }
}
