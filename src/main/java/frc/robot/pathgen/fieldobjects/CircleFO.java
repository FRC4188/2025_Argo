package frc.robot.pathgen.fieldobjects;

public class CircleFO extends FieldObject{
    protected float r = 0;

    CircleFO() {
        super(0,0);
    }

    CircleFO(float x, float y, float r) {
        super(x, y);
        this.r = r;
    }

    @Override
    public boolean touching_line(double x0, double y0, double x1, double y1) {
        double b = 2.f * ((x0 - x) * (x1 - x0) + (y0 - y) * (y1 - y0));
        double c = ((x0 - x) * (x0 - x) + (y0 - y) * (y0 - y) - r * r);
        double a = ((y1 - y0) * (y1 - y0) + (x1 - x0) * (x1 - x0));
    
        double discriminate = b * b - 4.f * a * c;
    
        if (discriminate < 0) { return false; }
        double val1 = (-b + Math.sqrt(discriminate)) / (2.f * a);
        double val2 = (-b - Math.sqrt(discriminate)) / (2.f * a);
        return ((0 <= val1 && val1 <= 1) || (0 <= val2 && val2 <= 1));
    }

    @Override
    public boolean touching_point(double x0, double y0) {
        return (x0 - x) * (x0 - x) + (y0 - y) * (y0 - y) <= r * r;
    }
    
}
