package frc.robot.pathgen;

public class PG_mathutils {
    /** 
     * clamps an integer value to min and max bounds
     * 
     * @param min minimum value
     * @param val value to be clamped
     * @param max maximum value
     * 
     * @return clamped value 
    */
    public static int clamp(int min, int val, int max) {
        if (val < min) return min;
        if (val > max) return max;

        return val;
    }

    /** 
     * clamps an short value to min and max bounds
     * 
     * @param min minimum value
     * @param val value to be clamped
     * @param max maximum value
     * 
     * @return clamped value 
    */
    public static short clamp(short min, short val, short max) {
        if (val < min) return min;
        if (val > max) return max;

        return val;
    }

    /** 
     * clamps an double value to min and max bounds
     * 
     * @param min minimum value
     * @param val value to be clamped
     * @param max maximum value
     * 
     * @return clamped value 
    */
    public static double clamp(double min, double val, double max) {
        if (val < min) return min;
        if (val > max) return max;

        return val;
    }

    /** 
     * returns the distance between two points
     * 
     * @param x0 x value of first point
     * @param y0 y value of first point
     * @param x1 x value of second point
     * @param y1 y value of second point
     * 
     * @return distance
    */
    public static double distancePoints(double x0, double y0, double x1, double y1) {
        return Math.hypot(x0 - x1, y0 - y1);
    }

    /** 
     * returns the distance between two points
     * 
     * @param x0 x value of first point
     * @param y0 y value of first point
     * @param x1 x value of second point
     * @param y1 y value of second point
     * 
     * @return distance
    */
    public static float distancePoints(float x0, float y0, float x1, float y1) {
        return (float)Math.hypot(x0 - x1, y0 - y1);
    }

    /** 
     * gets the shortest distance between a point and a line
     * 
     * @param x0 x value of first point of line
     * @param y0 y value of first point of line
     * @param x1 x value of second point of line
     * @param y1 y value of second point of line
     * @param xp x value of the point
     * @param yp y value of the point
     * 
     * @return shortest distance
    */
    public static double pointFromLine(double x0, double y0, double x1, double y1, double xp, double yp) {
        return Math.abs((x1 - x0) * yp - (y1 - y0) * xp - x1 * y0 + x0 * y1) / distancePoints(x0, y0, x1, y1);
    }

    /** 
     * checks if the two lines intersect
     * 
     * @param x1 x value of first point of first line
     * @param y1 y value of first point of first line
     * @param x2 x value of second point of first line
     * @param y2 y value of second point of first line
     * @param x3 x value of first point of second line
     * @param y3 y value of first point of second line
     * @param x4 x value of second point of second line
     * @param y4 y value of second point of second line
     * 
     * @return shortest distance
    */
    public static boolean linesIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
        double uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
    
        return uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1;
    }
}