package frc.robot.pathgen.fieldobjects;

import java.util.ArrayList;
import java.util.List;

public class FieldObjectHandler {
    private static FieldObjectHandler instance;

    public static synchronized FieldObjectHandler getInstance() {
        if (instance == null) instance = new FieldObjectHandler();
        return instance;
    }

    private List<FieldObject> fieldobjs;
    
    public FieldObjectHandler() {
        fieldobjs = new ArrayList<FieldObject>();
    }

    public void add(FieldObject fobj) {
        if (fieldobjs.contains(fobj)) {return;}

        fieldobjs.add(fobj);
    }

    public void remove(FieldObject fobj) {
        if (!fieldobjs.contains(fobj)) {return;}

        fieldobjs.remove(fieldobjs.indexOf(fobj));
    }

    public boolean any_touching_point(double x, double y) {
        for (FieldObject fobj: fieldobjs) {
            if (fobj.touching_point(x, y)) return true;
        }

        return false;
    }

    public boolean any_touching_line(double x0, double y0, double x1, double y1) {
        for (FieldObject fobj: fieldobjs) {
            if (fobj.touching_line(x0, y0, x1, y1)) return true;
        }

        return false;
    }



}
