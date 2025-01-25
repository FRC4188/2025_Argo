package frc.robot.pathgen;

import edu.wpi.first.math.geometry.Translation2d;

public class PG_math {
    public static short clamp(short min, short val, short max) {
	    if (val < min) return min;
	    if (val > max) return max;
	    return val;
    }

    public static float clamp(float min, float val, float max) {
	    if (val < min) return min;
	    if (val > max) return max;
	    return val;
    }

    public static float dot_f(Translation2d t1, Translation2d t2) {
	    return (float)(t1.getX() * t2.getX() + t1.getY() * t2.getY());
    }

    public static float point_from_lineseg_f(Translation2d l1, Translation2d l2, Translation2d point) {
	    Translation2d m = l2.minus(l1);
	    float l_sqrd = (float)(m.getX() * m.getX() + m.getY() * m.getY());
	    if (l_sqrd == 0.f) return (float)l1.getDistance(point);
	
	    float t = clamp(0, dot_f(point.minus(l1), m) / l_sqrd, 1);
	    Translation2d proj = l1.plus(m.times(t));
	    return (float)point.getDistance(proj);
    }

    private static boolean ccw(Translation2d a, Translation2d b, Translation2d c) {
        return (c.getY() - a.getY()) * (b.getX() - a.getX()) > (b.getY() - a.getY()) * (c.getX() - a.getX()); 
    };
    
    public static boolean intersect_lineseg(Translation2d l11, Translation2d l12, Translation2d l21, Translation2d l22) {
	    return ccw(l11, l21, l22) != ccw(l12, l21, l22) && ccw(l11, l12, l21) != ccw(l11, l12, l22);
    }   


}