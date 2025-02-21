package frc.robot.commands.superstructure.anglegen;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.autos.pathgen.PG_math;

import frc.robot.subsystems.scoring.superstructure.SuperConstraints;
import frc.robot.subsystems.scoring.superstructure.SuperState;

public class AngleGen {
    private static AngleGen instance;

    public static synchronized AngleGen getInstance() {
        if (instance == null) instance = new AngleGen(0.15f, 0); 
        return instance;
    }

    private Grid a_star;
    private float clearance = 0.0f;
    
	public AngleGen(float sample, float clearance) {
        this.clearance = clearance;

        a_star = new Grid(
            (float)SuperConstraints.WristConstraints.LOWEST_A,
            (float)SuperConstraints.WristConstraints.HIGHEST_A,
            (float)SuperConstraints.ArmConstraints.LOWEST_A,
            (float)SuperConstraints.ArmConstraints.HIGHEST_A,
            0,
            (float)SuperConstraints.ElevatorConstraints.RANGE,
            sample
        );

        update_obstacles();
    }

    public void set_clearance(float clearance) {
        this.clearance = clearance;
    }


    public void update_obstacles() {
        //inches because i pilled this from desmos and cad
        Translation2d b_tl = new Translation2d(-5.195177165, 18.24324);
        Translation2d b_tr = new Translation2d(-5.195177165, -18.24324);
        Translation2d b_br = new Translation2d(-9.132677165, -18.24324);
        Translation2d b_bl = new Translation2d(-9.132677165, 18.24324);

        Translation2d g_l = new Translation2d(-9.132677165,29.3367264837);
        Translation2d g_r = new Translation2d(-9.132677165,-29.3367264837);

        Translation2d c_tr = new Translation2d(4.5, -6.5);
        Translation2d c_tl = new Translation2d(4.5, 6.5);
        Translation2d c_br = new Translation2d(-4.5, -6.5);
        Translation2d c_bl = new Translation2d(-4.5, 6.5);

        for (short x = 0; x < a_star.length; x++) {
            for (short y = 0; y < a_star.width; y++) {
                for (short z = 0; z < a_star.height; z++) {
                SuperState pos = a_star.node_to_state(a_star.nodes[x][y][z]);

                Translation2d worigin = new Translation2d(Units.metersToInches(pos.getEleHeight()), 0).plus(
                    new Translation2d(16.1378264837, 0).rotateBy(Rotation2d.fromRadians(pos.getArmAngle())));
                
                Translation2d w_tr = worigin.plus(new Translation2d(13.1989, -9.443734).rotateBy(Rotation2d.fromRadians(pos.getGlobalAngle())));
                Translation2d w_br = worigin.plus(new Translation2d(2.619168, -9.443734).rotateBy(Rotation2d.fromRadians(pos.getGlobalAngle())));
                Translation2d w_tl = worigin.plus(new Translation2d(13.1989, 9.443734).rotateBy(Rotation2d.fromRadians(pos.getGlobalAngle())));
                Translation2d w_bl = worigin.plus(new Translation2d(2.619168, 9.443734).rotateBy(Rotation2d.fromRadians(pos.getGlobalAngle())));

                Translation2d ec_tr = c_tr.plus(new Translation2d(Units.metersToInches(pos.getEleHeight()), 0));
                Translation2d ec_tl = c_tl.plus(new Translation2d(Units.metersToInches(pos.getEleHeight()), 0));
                Translation2d ec_br = c_br.plus(new Translation2d(Units.metersToInches(pos.getEleHeight()), 0));
                Translation2d ec_bl = c_bl.plus(new Translation2d(Units.metersToInches(pos.getEleHeight()), 0));
                
                a_star.nodes[x][y][z].obstacle = 
                    a_star.nodes[x][y][z].obstacle ||
                    PG_math.lineseg_distance_lineseg(w_tr, w_br, ec_tr, ec_tl) <= clearance ||
                    PG_math.lineseg_distance_lineseg(w_tr, w_br, ec_br, ec_tr) <= clearance ||
                    PG_math.lineseg_distance_lineseg(w_tr, w_br, ec_bl, ec_br) <= clearance ||
                    PG_math.lineseg_distance_lineseg(w_tr, w_br, ec_tl, ec_tr) <= clearance ||
                    PG_math.lineseg_distance_lineseg(w_tl, w_bl, ec_tr, ec_tl) <= clearance ||
                    PG_math.lineseg_distance_lineseg(w_tl, w_bl, ec_br, ec_tr) <= clearance ||
                    PG_math.lineseg_distance_lineseg(w_tl, w_bl, ec_bl, ec_br) <= clearance ||
                    PG_math.lineseg_distance_lineseg(w_tl, w_bl, ec_tl, ec_tr) <= clearance ||
                    PG_math.lineseg_distance_lineseg(w_tr, w_br, b_tr, b_tl) <= clearance ||
                    PG_math.lineseg_distance_lineseg(w_tr, w_br, b_br, b_tr) <= clearance ||
                    PG_math.lineseg_distance_lineseg(w_tr, w_br, b_bl, b_tl) <= clearance ||
                    PG_math.lineseg_distance_lineseg(w_tr, w_br, g_l, g_r) <= clearance ||
                    PG_math.lineseg_distance_lineseg(w_tl, w_bl, b_tr, b_tl) <= clearance ||
                    PG_math.lineseg_distance_lineseg(w_tl, w_bl, b_br, b_tr) <= clearance ||
                    PG_math.lineseg_distance_lineseg(w_tl, w_bl, b_bl, b_tl) <= clearance ||
                    PG_math.lineseg_distance_lineseg(w_tl, w_bl, g_l, g_r) <= clearance;
                }
            }
        }    

    }

    public SuperTraj generateTrajectory(SuperState start, SuperState end) {
        ArrayList<SuperState> result = gen_pivots(start, end);

        return new SuperTraj(result, (result.size() - 1));
    }

    //TODO: reduce pivots to reduce awkward movement
	public ArrayList<SuperState> gen_pivots(SuperState start, SuperState end) {
        ArrayList<SuperState> pivots = new ArrayList<SuperState>();

        ArrayList<Node> backward_nodes = new ArrayList<Node>();

        if(!a_star.pathFind(a_star.state_to_node(start), a_star.state_to_node(end))) return pivots;

	    Node curNode = a_star.endNode;

        while (curNode != null) {
            backward_nodes.add(curNode);
            curNode = curNode.parent;
        }

        for (int i = backward_nodes.size() - 1; i >= 0; i--) {
		    pivots.add(a_star.node_to_state(backward_nodes.get(i)));
	    }
        
	    return pivots;
    }

    private static class Node {
        public short x = -1, y = -1, z = -1;
        public boolean obstacle = false, visited = false;
        public float fGlobalGoal = Float.MAX_VALUE, fLocalGoal = Float.MAX_VALUE;
    
        public List<Node> neighbours;
    
        public Node parent = null;
    
        public Node(short x, short y, short z) {
            this.x = x;
            this.y = y;
            this.z = z;
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
    
    private static class Grid {
        Node nodes[][][];
        short length = 0, width = 0, height = 0;
        float x_scale = 1, y_scale = 1, z_scale = 1;
        float x_offset = 0, y_offset = 0, z_offset = 0;
    
        Node startNode = null;
        Node endNode = null;
    
        Grid(float x_low, float x_high, float y_low, float y_high, float z_low, float z_high, float sample_size) {
            float m_length = x_high - x_low;
            float m_width = y_high - y_low;
            float m_height = z_high - z_low;
            x_scale = Math.min(sample_size, m_length);
            y_scale = Math.min(sample_size, m_width);
            z_scale = Math.min(sample_size, m_height);
    
            length = (short)(m_length / x_scale);
            width = (short)(m_width / y_scale);
            height = (short)(m_height / z_scale);
            
            x_scale = m_length / length;
            y_scale = m_width / width;
            z_scale = m_height / height;

            x_offset = x_low;
            y_offset = y_low;
            z_offset = z_low;
            
            createGrid(length, width, height);
        }

        boolean pathFind(Node start, Node end) {
    
            for (Node[][] layer : nodes) {
                for (Node[] line : layer) {
                    for (Node n : line) {
                        n.resetGoals();
                    }
                }
            }
    
            startNode = closet_clear(start);
            endNode = closet_clear(end);
        
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
    
            return endNode != null;
        }
    
        Node closet_clear(Node origin) {
            if (!origin.obstacle) return origin;
    
            ArrayList<Node> to_test_nodes = new ArrayList<>();
    
            to_test_nodes.add(origin);
    
            for (int i = 0; i < to_test_nodes.size(); i++) {
                for (Node n : to_test_nodes.get(i).neighbours) {
                    if (!n.obstacle) return n;
                    if (!to_test_nodes.contains(n)) to_test_nodes.add(n);
                }
            }
    
            return null;
        }
    
        Node getNode(short x, short y, short z) {
            x = PG_math.clamp((short)0, x, (short) (length - 1));
            y = PG_math.clamp((short)0, y, (short) (width - 1));
            z = PG_math.clamp((short)0, z, (short) (height - 1));
    
            return nodes[x][y][z];
        }
    
        void createGrid(short l, short w, short h) {
            width = w;
            length = l;
            height = h;

            nodes = new Node[l][w][h];
    
            for (short x = 0; x < l; x++) {
                for (short y = 0; y < w; y++) {
                    for (short z = 0; z < h; z++) {
                        nodes[x][y][z] = new Node(x, y, z);
                    }
                }
            }
            
            for (short x = 0; x < l; x++) {
                for (short y = 0; y < w; y++) {
                    for (short z = 0; z < h; z++) {
                        for (short nx = -1; nx < 2; nx++) {
                            for (short ny = -1; ny < 2; ny++) {
                                for (short nz = -1; nz < 2; nz++) {
                                    nodes[x][y][z].addNeighbour(getNode((short)(x + nx), (short)(y + ny), (short)(z + nz)));
                                }
                            }
                        }
                    }
                }
            }
        }

        float heuristic(Node a, Node b) {
            return (float)Math.hypot(a.z - b.z, Math.hypot(a.x - b.x, a.y - b.y));
        }
        
        SuperState node_to_state(Node node) {
            return new SuperState(
                x_scale * (node.x - 0.5) + x_offset, 
                y_scale * (node.y - 0.5) + y_offset, 
                z_scale * (node.z - 0.5) + z_offset);
        }

        Node state_to_node(SuperState pose) {
            return getNode(
                (short)((pose.getWristAngle() - x_offset) / x_scale + 1), 
                (short)((pose.getArmAngle() - y_offset) / y_scale + 1), 
                (short)((pose.getEleHeight() - z_offset) / z_scale + 1));
        }
    };
}