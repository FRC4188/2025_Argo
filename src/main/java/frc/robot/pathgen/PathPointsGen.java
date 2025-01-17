package frc.robot.pathgen;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.pathgen.fieldobjects.FieldObjectHandler;
import frc.robot.util.FieldConstant;

public class PathPointsGen {
    private static PathPointsGen instance;
    public static synchronized PathPointsGen getInstance() {
        if (instance == null) instance = new PathPointsGen(FieldConstant.field_length, FieldConstant.field_width, 0.1); //default sample size
        return instance;
    }

    private List<Translation2d> pivots = new ArrayList<Translation2d>();
    private Grid grid;
    private FieldObjectHandler fobjs = FieldObjectHandler.getInstance();

    //constructor
    public PathPointsGen(double length, double width, double sampleSize) {
        grid = new Grid((float)length, (float)width, (float)sampleSize);
    }

    //following are trajector generators, customized depending on the need or not for velocity directions

    //no velocity direction
    public Trajectory generateTrajectory(Translation2d start, Translation2d end, TrajectoryConfig config) {
        if (!generatePivots(start, end)) return new Trajectory();

        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(start, pivots.get(0).minus(start).getAngle()), 
            pivots, 
            new Pose2d(end, end.minus(pivots.get(pivots.size() - 1)).getAngle()), 
            config);
    }

    //start velocity direction
    public Trajectory generateTrajectory(Pose2d start, Translation2d end, TrajectoryConfig config) {
        if (!generatePivots(start.getTranslation(), end)) return new Trajectory();
        
        return TrajectoryGenerator.generateTrajectory(
            start, 
            pivots, 
            new Pose2d(end, end.minus(pivots.get(pivots.size() - 1)).getAngle()), 
            config);
    }

    //end velocity direction
    public Trajectory generateTrajectory(Translation2d start, Pose2d end, TrajectoryConfig config) {
        if (!generatePivots(start, end.getTranslation())) return new Trajectory();

        return TrajectoryGenerator.generateTrajectory(
            new Pose2d(start, pivots.get(0).minus(start).getAngle()), 
            pivots, 
            end,
            config);
    }

    //both velocity directions
    public Trajectory generateTrajectory(Pose2d start, Pose2d end, TrajectoryConfig config) {
        if (!generatePivots(start.getTranslation(), end.getTranslation())) return new Trajectory();

        return TrajectoryGenerator.generateTrajectory(
            start, 
            pivots, 
            end, 
            config);
    }

    //generates pivots based on A star grid path finding
    private boolean generatePivots(Translation2d start, Translation2d end) {
        grid.updateGrid();
        pivots.clear();

        List<Node> backwardPivots = new ArrayList<Node>();

        if (!grid.pathFind(grid.t2d_to_node(start), grid.t2d_to_node(end))) { return false;}

        //start and end for the first initial pivots points
        backwardPivots.add(grid.endNode);
        backwardPivots.add(grid.startNode);

        Node curNode = null;
        Node pivotNode = null;
        int curIndex = 0;

        //will check if a line between two pivots points crosses an obstacle
        //if false continue to next pair
        //if true create a pivot point between those two points
        while (curIndex < backwardPivots.size() - 1) {

            curNode = backwardPivots.get(curIndex).parent;

            if (any_touching_line(backwardPivots.get(curIndex), backwardPivots.get(curIndex + 1))) {
                double maxD = -1;

                while (curNode != backwardPivots.get(curIndex + 1)) {
                    double d = pointFromLine(backwardPivots.get(curIndex), backwardPivots.get(curIndex + 1), curNode);

                    if (maxD < d) {
                        maxD = d;
                        pivotNode = curNode;
                    }

                    curNode = curNode.parent;
                }

                if (pivotNode != null) {
                    backwardPivots.add(curIndex + 1, pivotNode);
                } else {
                    curIndex++;
                }

            } else {
                curIndex++;
            }
        }

        //started from end to start node so we have to reverse those pivots
        //convert nodes to translation2ds
        Collections.reverse(backwardPivots);

        for (Node n : backwardPivots) {pivots.add(grid.node_to_t2d(n));}   
        
        //trajectory generator does not require start and end interior waypoints so we wipe start and end points
        if (pivots.size() < 2)  return false; 
        pivots.remove(0);
        pivots.remove(pivots.size() - 1);

        //if our only pivots points were the start and end points that we just wiped we will interpolate and have a midpoint
        if (pivots.isEmpty()) {
            pivots.add(new Translation2d(0.5 * (start.getX() + end.getX()), 0.5 * (start.getY() + end.getY())));
        }

        return true;
    }

    private boolean any_touching_line(Node n1, Node n2) {
        return fobjs.any_touching_line(
            grid.node_to_t2d(n1).getX(), grid.node_to_t2d(n1).getY(), 
            grid.node_to_t2d(n2).getX(), grid.node_to_t2d(n2).getY()
        );
    }

    private double pointFromLine(Node n1, Node n2, Node point) {
        return PG_mathutils.pointFromLine(
            grid.node_to_t2d(n1).getX(), grid.node_to_t2d(n1).getY(), 
            grid.node_to_t2d(n2).getX(), grid.node_to_t2d(n2).getY(),
            grid.node_to_t2d(point).getX(), grid.node_to_t2d(point).getY()  
        );
    }
}

//A star grid calculation
//classes in here because its the background of the PathPointsGen
//not needed to be accessed anywhere outside this file

class Node {
    final public short x, y;
    public List<Node> neighbours;
    public boolean obstacle = false, visited = false;
    public float fGlobalGoal, fLocalGoal;
    public Node parent;

    //constructor
    public Node(short x, short y) {
        this.x = x;
        this.y = y;
        neighbours = new ArrayList<Node>();
        resetGoals();
    }

    //grid construction
    public void addNeighbour(Node other) {
        if (!neighbours.contains(other) && other != this) {
            neighbours.add(other);
        }
    }

    //reset goals for pathfinding
    public void resetGoals() {
        parent = null;
        visited = false;
        fGlobalGoal = Float.MAX_VALUE;
        fLocalGoal = Float.MAX_VALUE;
    }
}   

class Grid {
    private Node[] nodes = null;

    private short length, width;
    private float x_scale, y_scale;

    public Node startNode = null;
    public Node endNode = null;

    //constructor
    //sampleSize is resized to make integer divions in length and width
    public Grid(float length, float width, float sampleSize) {
        x_scale = Math.min(sampleSize, length);
        y_scale = Math.min(sampleSize, width);

        this.length = (short) (length / x_scale);
        this.width = (short) (width / y_scale);

        x_scale = length / this.length;
        y_scale = width / this.width;

        createGrid(this.length, this.width);
        updateGrid();
    }

    //creates the grids with nodes
    private void createGrid(short l, short w) {
        width = w;
        length = l;
        nodes = new Node[w  * l];

        for (short x = 0; x < l; x++) {
            for (short y = 0; y < w; y++) {
                nodes[y * l + x] = new Node(x, y);
            }
        }

        for (short x = 0; x < l; x++) {
            for (short y = 0; y < w; y++) {
                for (short nx = -1; nx < 2; nx++) {
                    for (short ny = -1; ny < 2; ny++) {
                        nodes[y * l + x].addNeighbour(getNode(x + nx, y + ny));
                    }
                }
            }
        }
    }

    //updates the grid for any new obstacles
    public void updateGrid() {
        for (short x = 0; x < length; x++) {
            for (short y = 0; y < width; y++) {
                nodes[y * length + x].obstacle = (FieldObjectHandler.getInstance().any_touching_point((x + 0.5) * x_scale, (y + 0.5) * y_scale));
            }
        }
    }

    //path generation; returns if path is found
    public boolean pathFind(Node start, Node end) {
        for (Node n: nodes) {
            n.resetGoals();
        }

        startNode = start;
        endNode = end;

        if (startNode == null || endNode == null) return false;

        Node curNode = startNode;
        startNode.fLocalGoal = 0.0f;
        startNode.fGlobalGoal = heuristic(startNode, endNode);

        List<Node> listNotTestedNodes = new ArrayList<Node>();
        listNotTestedNodes.add(startNode);

        while (!listNotTestedNodes.isEmpty() && curNode != endNode) {
            listNotTestedNodes.sort((Node lhs, Node rhs) -> Double.compare(lhs.fGlobalGoal, rhs.fGlobalGoal));

            while (!listNotTestedNodes.isEmpty() && listNotTestedNodes.get(0).visited) {
                    listNotTestedNodes.remove(0);
            }

            if (listNotTestedNodes.isEmpty()) break;

            curNode = listNotTestedNodes.get(0);
            curNode.visited = true;

            for (Node nodeNeighbour : curNode.neighbours) {
                if (!nodeNeighbour.visited && !nodeNeighbour.obstacle) listNotTestedNodes.add(nodeNeighbour);

                float fPossiblyLowerGoal = curNode.fLocalGoal + heuristic(curNode, nodeNeighbour);

                if (fPossiblyLowerGoal < nodeNeighbour.fLocalGoal) {
                    nodeNeighbour.parent = curNode;
                    nodeNeighbour.fLocalGoal = fPossiblyLowerGoal;
                    nodeNeighbour.fGlobalGoal = nodeNeighbour.fLocalGoal + heuristic(nodeNeighbour, endNode);
                }

            }
        }
        return endNode.parent != null;
    }

    //utility for pathfinding
    private float heuristic(Node a, Node b) {
        return PG_mathutils.distancePoints(a.x, a.y, b.x, b.y);
    }
    
    public Node getNode(int x, int y) {
        x = PG_mathutils.clamp(0, x, length - 1);
        y = PG_mathutils.clamp(0, y, width - 1);
        return nodes[x + y * length];
    }

    private boolean neighbour_obstacles(Node n) {
        for (Node neighbour : n.neighbours) {
            if (neighbour.obstacle) return true;
        }

        return false;
    }

    public Translation2d node_to_t2d(Node node) {
        return new Translation2d(x_scale * (node.x - 0.5), y_scale * (node.x - 0.5));
    }

    public Node t2d_to_node(Translation2d t2d) {
        return getNode((int) (t2d.getX()/x_scale + 1), (int) (t2d.getY()/y_scale + 1));
    }
}