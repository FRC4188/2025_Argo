package frc.robot.pathgen.fieldobjects;

import frc.robot.pathgen.PG_mathutils;

public class RectFO extends FieldObject {
    private float w = 0;
    private float h = 0;
    
    RectFO() {
        super(0,0);
    }

    RectFO(float x, float y, float w, float h) {
        super(x, y);
        this.w = w;
        this.h = h;
    }

    @Override
    public boolean touching_point(double x0, double y0) {
        return (x - 0.5 * w <= x0 && x0 <= x + 0.5 * w && y - 0.5 * h <= y0 && y0 <= y + 0.5 * h);
    }

    @Override
    public boolean touching_line(double x0, double y0, double x1, double y1) {
        return PG_mathutils.linesIntersect(x0, y0, x1, y1, x - w * 0.5, y - h * 0.5, x + w * 0.5, y - h * 0.5) ||
        PG_mathutils.linesIntersect(x0, y0, x1, y1, x + w * 0.5, y - h * 0.5, x + w * 0.5, y + h * 0.5) ||
        PG_mathutils.linesIntersect(x0, y0, x1, y1, x + w * 0.5, y + h * 0.5, x - w * 0.5, y + h * 0.5) ||
        PG_mathutils.linesIntersect(x0, y0, x1, y1, x - w * 0.5, y + h * 0.5, x - w * 0.5, y - h * 0.5);
    }

}
