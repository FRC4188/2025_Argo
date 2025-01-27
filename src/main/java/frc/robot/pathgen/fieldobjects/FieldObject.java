package frc.robot.pathgen.fieldobjects;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.pathgen.PG_math;

public abstract class FieldObject {
    protected float c_x = 0;
    protected float c_y = 0;

    public FieldObject() {}

    public FieldObject(float x, float y) {
        c_x = x;
        c_y = y;
        FOHandler.getInstance().addFO(this);
    }

    public boolean touching_line(Translation2d l1, Translation2d l2) {
        return PG_math.point_from_lineseg_f(l1, l2, new Translation2d(c_x, c_y)) == 0;
    }
    
    public boolean touching_point(Translation2d point) {
        return c_x == point.getX() && c_y == point.getY();
    }
}
