package frc.robot.pathgen.fieldobjects;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.pathgen.PG_math;

public class PolygonFO extends FieldObject{
    Translation2d[] vertices;

    public PolygonFO() {}

    public PolygonFO(Translation2d... vertices) {
        super(-1, -1);
        this.vertices = vertices;
    }

    @Override
    public float from_line(Translation2d l1, Translation2d l2) {
        float distance = Float.MAX_VALUE;

	    for (int i = 0; i < vertices.length; i++) {
            float dis = PG_math.lineseg_distance_lineseg(vertices[i], vertices[(i + 1)%vertices.length], l1, l2);
		    if (dis < distance) {
                distance = dis;
            } 
	    }
	    return distance;
    }

    @Override
    public float from_point(Translation2d point) {
	    float distance = Float.MAX_VALUE;

	    for (int i = 0; i < vertices.length; i++) {
            float dis = PG_math.point_from_lineseg_f(vertices[i], vertices[(i + 1)%vertices.length], point);
		    if (dis < distance) {
                distance = dis;
            } 
	    }
	    return distance;
    }
}
