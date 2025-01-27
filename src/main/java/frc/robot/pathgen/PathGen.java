package frc.robot.pathgen;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.pathgen.fieldobjects.FOHandler;
import frc.robot.util.FieldConstant;

public class PathGen {
    private static PathGen instance;
    public static synchronized PathGen getInstance() {
        if (instance == null) instance = new PathGen((float)FieldConstant.field_length, (float)FieldConstant.field_width, 0.25f); //default sample size
        return instance;
    }

    private Grid a_star;
	
    public PathGen() {

    }

	public PathGen(float length, float width, float sample) {
        a_star = new Grid(length, width, sample);
    }

    public Trajectory generateTrajectory(Translation2d start, Translation2d end, TrajectoryConfig config) {
        ArrayList<Translation2d> pivots = gen_pivots(start, end);
        
        if (pivots.isEmpty()) return new Trajectory();

        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(start, pivots.get(0).minus(start).getAngle()), 
            pivots, 
            new Pose2d(end, end.minus(pivots.get(pivots.size() - 1)).getAngle()), 
            config);
    }

    //start velocity direction
    public Trajectory generateTrajectory(Pose2d start, Translation2d end, TrajectoryConfig config) {
        ArrayList<Translation2d> pivots = gen_pivots(start.getTranslation(), end);
        
        if (pivots.isEmpty()) return new Trajectory();

        return TrajectoryGenerator.generateTrajectory(
            start, 
            pivots, 
            new Pose2d(end, end.minus(pivots.get(pivots.size() - 1)).getAngle()), 
            config);
    }

    //end velocity direction
    public Trajectory generateTrajectory(Translation2d start, Pose2d end, TrajectoryConfig config) {
        ArrayList<Translation2d> pivots = gen_pivots(start, end.getTranslation());

        if (pivots.isEmpty()) return new Trajectory();

        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(start, pivots.get(0).minus(start).getAngle()), 
            pivots, 
            end,
            config);
    }

    //both velocity directions
    public Trajectory generateTrajectory(Pose2d start, Pose2d end, TrajectoryConfig config) {
        ArrayList<Translation2d> pivots = gen_pivots(start.getTranslation(), end.getTranslation());

        if (pivots.isEmpty()) return new Trajectory();

        return TrajectoryGenerator.generateTrajectory(
            start, 
            pivots, 
            end, 
            config);
    }

	public ArrayList<Translation2d> gen_pivots(Translation2d start, Translation2d end) {
        ArrayList<Translation2d> pivots = new ArrayList<Translation2d>();

	    ArrayList<Node> backward_nodes = new ArrayList<Node>();

	    if (!a_star.pathFind(a_star.t2d_to_node(start), a_star.t2d_to_node(end))) return pivots;
        
	    backward_nodes.add(a_star.t2d_to_node(end));
	    backward_nodes.add(a_star.t2d_to_node(start));

	    Node curNode = null;
	    Node pivotNode = null;
	    short curIndex = 0;

	    while (curIndex < backward_nodes.size() - 1) {

		    curNode = backward_nodes.get(curIndex).parent;

		    if (FOHandler.getInstance().any_touching_line(a_star.node_to_t2d(backward_nodes.get(curIndex)), a_star.node_to_t2d(backward_nodes.get(curIndex + 1)))) {
			
			    float maxD = -1;

			    while (curNode != backward_nodes.get(curIndex + 1)) {
				    float d = PG_math.point_from_lineseg_f(
					    a_star.node_to_t2d(backward_nodes.get(curIndex)), 
					    a_star.node_to_t2d(backward_nodes.get(curIndex + 1)), 
					    a_star.node_to_t2d(curNode));

				    if (maxD < d) {
					    maxD = d;
					    pivotNode = curNode;
				    }

				    curNode = curNode.parent;

			    }

			    if (pivotNode != null) {
				    backward_nodes.add(curIndex + 1, pivotNode);
			    }   
			    else {
				    curIndex++;
			    }
			    pivotNode = null;
		    }
		    else {
			    curIndex++;
		    }
	    }

        
	    for (int i = backward_nodes.size() - 1; i >= 0; i--) {
		    pivots.add(a_star.node_to_t2d(backward_nodes.get(i)));
	    }

	    if (pivots.size() < 2) return pivots;
	    pivots.remove(0);
	    pivots.remove(pivots.size() - 1);
        
	    if (pivots.size() == 0) {
		    pivots.add(end.plus(start.minus(end).times(0.5)));
	    }
	

	    return pivots;
    }

	public void update_grid_fo(float robot_radius) {
        a_star.update_fo(robot_radius);
    }
}

//A star grid calculation
//classes in here because its the background of the PathPointsGen
//not needed to be accessed anywhere outside this file

class Node {
	public short x = -1, y = -1;
	public boolean obstacle = false, visited = false;
	public float fGlobalGoal = Float.MAX_VALUE, fLocalGoal = Float.MAX_VALUE;

	public List<Node> neighbours;

	public Node parent = null;

	public Node() {

    }

	public Node(short x, short y) {
        this.x = x;
        this.y = y;
        neighbours = new ArrayList<Node>();
    }

	public void addNeighbour(Node other) {
        if (!neighbours.contains(other) && other != this) {neighbours.add(other);}
    }
	public void resetGoals() {
        parent = null;
        visited = false;
        fGlobalGoal = Float.MAX_VALUE;
        fLocalGoal = Float.MAX_VALUE;
    }
};

class Grid {
	Node nodes[];
	short length = 0, width = 0;
	float x_scale = 1, y_scale = 1;

	public Node startNode = null;
	public Node endNode = null;

	public Grid() {

    }
	public Grid(float m_length, float m_width, float sample_size) {
        x_scale = Math.min(sample_size, m_length);
        y_scale = Math.min(sample_size, m_width);

        length = (short)(m_length / x_scale);
        width = (short)(m_width / y_scale);
        
        x_scale = m_length / length;
        y_scale = m_width / width;
    
        
        createGrid(length, width);
    }

	public void update_fo(float robot_radius) {
        for (int x = 0; x < length; x++) {
            for (int y = 0; y < width; y++) {
                Translation2d pos = node_to_t2d(nodes[y * length + x]);

                Translation2d q1 = new Translation2d(pos.getX() +  robot_radius, pos.getY() +  robot_radius);
                Translation2d q2 = new Translation2d(pos.getX() - robot_radius, pos.getY() +  robot_radius);
                Translation2d q3 = new Translation2d(pos.getX() - robot_radius, pos.getY() - robot_radius);
                Translation2d q4 = new Translation2d(pos.getX() + robot_radius, pos.getY() - robot_radius);
    
                nodes[y * length + x].obstacle = (
                    FOHandler.getInstance().any_touching_line(q1, q2) || 
                    FOHandler.getInstance().any_touching_line(q2, q3) || 
                    FOHandler.getInstance().any_touching_line(q3, q4) || 
                    FOHandler.getInstance().any_touching_line(q4, q1));
            }
        }
    }
	public boolean pathFind(Node start, Node end) {

        for (int i = 0; i < width * length; i++) {
            nodes[i].resetGoals();
        }
    
        startNode = start;
        endNode = end;
    
        if (startNode == null || endNode == null) return false;
    
        Node curNode = startNode;
        startNode.fLocalGoal = 0.0f;
        startNode.fGlobalGoal = heuristic(startNode, endNode);
    
        List<Node> listNotTestedNodes = new ArrayList<Node>();
        listNotTestedNodes.add(startNode);
    
        while (!listNotTestedNodes.isEmpty() && curNode != endNode)
        {
            listNotTestedNodes.sort((Node lhs, Node rhs) -> Double.compare(lhs.fGlobalGoal, rhs.fGlobalGoal));

    
            while (!listNotTestedNodes.isEmpty() && listNotTestedNodes.get(0).visited)
                listNotTestedNodes.remove(0);
    
            if (listNotTestedNodes.isEmpty())
                break;
    
            curNode = listNotTestedNodes.get(0);
            curNode.visited = true;
    
            for (Node nodeNeighbour : curNode.neighbours)
            {
                if (!nodeNeighbour.visited && !nodeNeighbour.obstacle)
                    listNotTestedNodes.add(nodeNeighbour);
    
                float fPossiblyLowerGoal = curNode.fLocalGoal + heuristic(curNode, nodeNeighbour);
    
                if (fPossiblyLowerGoal < nodeNeighbour.fLocalGoal)
                {
                    nodeNeighbour.parent = curNode;
                    nodeNeighbour.fLocalGoal = fPossiblyLowerGoal;
                    nodeNeighbour.fGlobalGoal = nodeNeighbour.fLocalGoal + heuristic(nodeNeighbour, endNode);
                }
            }
        }
    
        return endNode.parent != null;
    }

	public Node getNode(short x, short y) {
        x = PG_math.clamp((short)0, x, (short) (length - 1));
	    y = PG_math.clamp((short)0, y, (short) (width - 1));

	    return nodes[y * length + x];
    }

	private void createGrid(short l, short w) {
        
        width = w;
	    length = l;
	    nodes = new Node[w * l];

	    for (short x = 0; x < l; x++) {
		    for (short y = 0; y < w; y++) {
			    nodes[y * l + x] = new Node(x, y);
		    }
	    }
        
	    for (short x = 0; x < l; x++) {
		    for (short y = 0; y < w; y++) {
			    for (short nx = -1; nx < 2; nx++) {
				    for (short ny = -1; ny < 2; ny++) {
					    nodes[y * l + x].addNeighbour(getNode((short)(x + nx), (short)(y + ny)));
				    }
			    }
		    }
	    }
    }
	private float heuristic(Node a, Node b) {
        return (float)Math.hypot(a.x - b.x, a.y - b.y);
    }
	
	public Translation2d node_to_t2d(Node node) {
        return new Translation2d(x_scale * (node.x - 0.5), y_scale * (node.y - 0.5));
    }
	public Node t2d_to_node(Translation2d pose) {
        return getNode((short)(pose.getX() / x_scale + 1), (short)(pose.getY() / y_scale + 1));
    }
};