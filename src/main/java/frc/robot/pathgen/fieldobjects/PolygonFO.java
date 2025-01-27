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

    public boolean touching_line(Translation2d l1, Translation2d l2) {
	    for (int i = 0; i < vertices.length; i++) {
		    if (PG_math.intersect_lineseg(vertices[i], vertices[(i + 1)%vertices.length], l1, l2)) return true;
	    }
	    return false;
    }

    public boolean touching_point(Translation2d point) {
	for (int i = 0; i < vertices.length; i++) {
		if (PG_math.point_from_lineseg_f(vertices[i], vertices[(i + 1)%vertices.length], point) == 0) return true;
	}
	return false;
}
}
