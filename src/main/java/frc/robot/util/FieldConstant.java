package frc.robot.util;


import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

//in meters/coordinates
//all distances are measured from center to center
public class FieldConstant {
    public static double field_length = Units.inchesToMeters(690.875);
    public static double field_width = Units.inchesToMeters(317);
    public static double field_midX = field_length /2;
    public static double field_midY = field_width / 2;

    public static double coral_length = Units.inchesToMeters(11.875);
    public static double algae_radius = Units.inchesToMeters(41) / 2;
    public static double algae_radius_offset = Units.inchesToMeters(0.25);

    public class Cage {
        public static double cage_height = Units.inchesToMeters(24);
        public static double cage_width = Units.inchesToMeters(7.375);
        public static double deep_height_from_bottom = Units.inchesToMeters(3.50);
        public static double shallow_height_from_bottom = Units.inchesToMeters(29.375);

        public static Translation3d[] cage_positions = new Translation3d[6];

        static {
            for(int i = 0; i < 3; i ++){
                cage_positions[i] = new Translation3d(
                    field_midX, 
                    field_midY + Units.inchesToMeters(41.5) + (1.01 * i),
                    shallow_height_from_bottom);
            }

            for(int i = 3; i < 6; i ++){
                cage_positions[i] = new Translation3d(
                    field_midX, 
                    field_midY - Units.inchesToMeters(41.5) - (1.01 * (i-3)),
                    shallow_height_from_bottom);
            }
        }

        public void setDeep(int location, boolean isDeep){
            Translation3d cage = cage_positions[location];

            if(isDeep)
                cage_positions[location] = new Translation3d(cage.getX(), cage.getY(), deep_height_from_bottom);
            else
                cage_positions[location] = new Translation3d(cage.getX(), cage.getY(), shallow_height_from_bottom);
        }
    }

    public class Reef{
        public static double dis_from_DS = Units.inchesToMeters(144.00);
        public static double side_to_side = Units.inchesToMeters(93.50 - (14.00 * 2));
        public static double mid_to_rail = field_width / 2;
        public static double corner_to_opposite = (side_to_side / Math.sqrt(3)) * 2;  
        public static double base_side = corner_to_opposite /2; 
        
        //distance from corner to center
        public static double dx_from_mid = side_to_side / 2;
        public static double dy_fromm_mid  = Rotation2d.fromDegrees(30).getSin() * (corner_to_opposite / 2);

        public static double branch_distance = Units.inchesToMeters(13);
        public static double branch_inset = Units.inchesToMeters(1.625);
        public static double top_inset = Units.inchesToMeters(1.125);


        public static double L1_front_height = Units.inchesToMeters(18);
        public static double L2_highest_h = Units.inchesToMeters(31.875);
        public static double L3_highest_h = Units.inchesToMeters(47.625);
        public static double L4_highest_h = Units.inchesToMeters(72.00);

        public class Base{
            //corners
            public static Translation2d center = new Translation2d(dis_from_DS + side_to_side / 2, mid_to_rail);

            public static Translation2d right_wall_corner = new Translation2d(dis_from_DS, mid_to_rail - dy_fromm_mid);
            public static Translation2d left_wall_corner = new Translation2d(dis_from_DS, mid_to_rail + dy_fromm_mid);

            public static Translation2d left_barge_corner = new Translation2d(dis_from_DS + side_to_side, mid_to_rail + dy_fromm_mid);
            public static Translation2d right_barge_corner = new Translation2d(dis_from_DS - side_to_side, mid_to_rail - dy_fromm_mid);

            public static Translation2d source_corner = new Translation2d(dis_from_DS + dx_from_mid, mid_to_rail + corner_to_opposite / 2);
            public static Translation2d processor_corner = new Translation2d(dis_from_DS + dx_from_mid, mid_to_rail - corner_to_opposite / 2);
            
        }

        //i dont want to go thru this hell HELL NO - lyad
        //i did it anyways :P z value is a rough estimamte, assuming 
        public class Algae{
            public static Translation3d wall_algae = new Translation3d(dis_from_DS + algae_radius, Base.right_wall_corner.getY() + base_side /2, L3_highest_h + algae_radius);
            public static Translation3d left_wall_algae = new Translation3d(dis_from_DS + (side_to_side / 4), Base.left_wall_corner.getY() + base_side / 4, L2_highest_h + algae_radius);
            public static Translation3d right_wall_algae = new Translation3d(dis_from_DS + (side_to_side / 4), Base.right_wall_corner.getY() - base_side / 4, L2_highest_h + algae_radius);
            public static Translation3d left_barge_algae = new Translation3d(dis_from_DS + (side_to_side / 4 * 3), Base.left_barge_corner.getY() + base_side / 4, L3_highest_h + algae_radius);
            public static Translation3d right_barge_algae = new Translation3d(dis_from_DS + (side_to_side / 4 * 3), Base.right_barge_corner.getY() - base_side / 4, L3_highest_h + algae_radius);
            public static Translation3d barge_algae = new Translation3d(dis_from_DS + side_to_side - algae_radius, Base.right_wall_corner.getY() + base_side / 2, L2_highest_h + algae_radius);
        }
    }

    public class Elem_Locations{
        public static double dis_from_DS = Units.inchesToMeters(48);
        public static double dis_apart = Units.inchesToMeters(72);
        public static Translation2d[] corals_locations = new Translation2d[3];
        public static Translation3d[] algae_locations = new Translation3d[3];

        static{
            for(int i = 0; i < 3; i ++){
                corals_locations[i] = new Translation2d(dis_from_DS, ((field_width / 2) + dis_apart) - dis_apart * i);
                algae_locations[i] = new Translation3d(corals_locations[i].getX(), corals_locations[i].getY(), coral_length);
            }
        }
                
    }

    public class Processor{
        public static double width_opening = Units.inchesToMeters(28);
        public static double height_opening = Units.inchesToMeters(20);
        public static double height = Units.inchesToMeters(7);
    }

    public class Source{
        //TODO: research where origin (0,0) start -> alliance wall or right corner of blue coral station area 
        public static double bottom_opening_height = Units.inchesToMeters(25.5);

        public static double source_length = Units.inchesToMeters(79.268);

        //rotat is angle of robot facing da source
        public static Translation2d bottom_source_alliance = new Translation2d(Units.inchesToMeters(15.819), Units.inchesToMeters(35.809));
        public static Translation2d bottom_source_mid = new Translation2d(Units.inchesToMeters(31.863), Units.inchesToMeters(23.8725));
        public static Translation2d bottom_source_reef = new Translation2d(Units.inchesToMeters(47.457), Units.inchesToMeters(11.936));
        public static Rotation2d bottom_source_rotat = new Rotation2d(Degree.of(52.9636));

        public static Translation2d top_source_alliance = new Translation2d(bottom_source_alliance.getX(), AllianceFlip.flipY(bottom_source_alliance.getY()));
        public static Translation2d top_source_mid = new Translation2d(bottom_source_mid.getX(), AllianceFlip.flipY(bottom_source_mid.getY()));
        public static Translation2d top_source_reef = new Translation2d(bottom_source_reef.getX(), AllianceFlip.flipY(bottom_source_reef.getY()));
        public static Rotation2d top_source_rotat = new Rotation2d(Degree.of(-52.9636));      
    }

}
