package frc.robot.pathgen.fieldobjects;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.pathgen.PG_math;

public class RectFO extends FieldObject {   
    private float length = 0;
    private float width = 0;
    
    RectFO() {

    }

    RectFO(float x, float y, float l, float w) {
        super(x, y);
	    length = l;
	    width = w;
    }

    public boolean touching_point(Translation2d point) {
	    return (point.getX() >= c_x - 0.5 * length && point.getX() <= c_x + 0.5 * length &&
		    point.getY() >= c_y - 0.5 * width && point.getY() <= c_y + 0.5 * width);
    }

    public boolean touching_line(Translation2d l1, Translation2d l2) {
	    Translation2d q1 = new Translation2d(c_x + 0.5 * length, c_y + 0.5 * width);
	    Translation2d q2 = new Translation2d(c_x - 0.5 * length, c_y + 0.5 * width);
	    Translation2d q3 = new Translation2d(c_x - 0.5 * length, c_y - 0.5 * width);
	    Translation2d q4 = new Translation2d(c_x + 0.5 * length, c_y - 0.5 * width);

	    return (PG_math.intersect_lineseg(l1, l2, q1, q2) || PG_math.intersect_lineseg(l1, l2, q2, q3) || PG_math.intersect_lineseg(l1, l2, q3, q4) || PG_math.intersect_lineseg(l1, l2, q4, q1));
    }

}
