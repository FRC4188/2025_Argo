package frc.robot.commands.autos.pathgen.fieldobjects;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

public class FOHandler {
    public List<FieldObject> fobjs;
    
    public FOHandler() {
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
    
    public float shortest_from_line(Translation2d l1, Translation2d l2) {
        float distance = Float.MAX_VALUE;

        for (FieldObject fo : fobjs) {
            float dis = fo.from_line(l1, l2);
            if (dis < distance) distance = dis;
        }
        return distance;
    }
    
    public float shortest_from_point(Translation2d point) {
        float distance = Float.MAX_VALUE;

        for (FieldObject fo : fobjs) {
            float dis = fo.from_point(point);
            if (dis < distance) distance = dis;
        }
        return distance;
    }

    public void clear() {
        fobjs.clear();
    }
}