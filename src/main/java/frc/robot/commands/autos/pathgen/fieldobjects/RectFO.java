package frc.robot.commands.autos.pathgen.fieldobjects;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.autos.pathgen.PG_math;

public class RectFO extends PolygonFO {   

    RectFO() {

    }

    public RectFO(float x, float y, float l, float w) {
        super(
            true,
            new Translation2d(x + 0.5 * l, y + 0.5 * w), 
            new Translation2d(x + 0.5 * l, y + 0.5 * w), 
            new Translation2d(x - 0.5 * l, y - 0.5 * w), 
            new Translation2d(x + 0.5 * l, y - 0.5 * w)
        );
    }

}
