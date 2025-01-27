package frc.robot.pathgen.fieldobjects;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

public class FOHandler {
    private static FOHandler instance;

    public static synchronized FOHandler getInstance() {
        if (instance == null) instance = new FOHandler();
        return instance;
    }

    private List<FieldObject> fobjs;
    
    private FOHandler() {
        fobjs = new ArrayList<FieldObject>();
    }

    public void addFO(FieldObject fo) {
        removeFO(fo);
    
        fobjs.add(fo);
    }
    
    public void removeFO(FieldObject fo) {
        int index = fobjs.indexOf(fo);
        if (index > -1) fobjs.remove(index);
    }
    
    public boolean any_touching_line(Translation2d l1, Translation2d l2) {
        for (FieldObject fo : fobjs) {
            if (fo.touching_line(l1, l2)) return true;
        }
        return false;
    }
    
    public boolean any_touching_point(Translation2d point) {
        for (FieldObject fo : fobjs) {
            if (fo.touching_point(point)) return true;
        }
        return false;
    }

    public void clear() {
        fobjs.clear();
    }
}