package frc.robot.pathgen.fieldobjects;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.pathgen.PG_mathutils;

public abstract class FieldObject {
    protected float x = 0;
    protected float y = 0;

    public FieldObject() {}

    public FieldObject(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public boolean touching_point(double x, double y) {
        return this.x  == x && this.y == y;
    }

    public boolean touching_line(double x0, double y0, double x1, double y1) {
        return PG_mathutils.pointFromLine(x0, y0, x1, y1, x, y) == 0;
    }
}
