package frc.robot.util;


import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

//in meters/coordinates
//all distances are measured from center to center

//All rotations are outward perpendicular to the line
//Degrees; [-180, 180)
public class FieldConstant {

    //Field Carpet
    public static double field_length = Units.inchesToMeters(690.875);
    public static double field_width = Units.inchesToMeters(317);
    public static double field_center_x = field_length /2;
    public static double field_center_y = field_width / 2;

    //Element Dimensions
    public static double coral_length = Units.inchesToMeters(11.875);
    public static double coral_diameter = Units.inchesToMeters(4.5);
    public static double algae_radius = Units.inchesToMeters(8.125);
    public static double algae_tolerance = Units.inchesToMeters(0.125);

    public class Cage {
        //dimensions
        public static double cage_height = Units.inchesToMeters(24);
        public static double cage_width = Units.inchesToMeters(7.375);

        //height from bottom of cage to carpet
        public static double deep_height = Units.inchesToMeters(3.125);
        public static double shallow_height = Units.inchesToMeters(30.125);


        //ordered in distance to scoring table; 0 - closest, 5 - farthest
        public static Translation3d[] cage_positions = new Translation3d[6];

        static {
            cage_positions[0] = new Translation3d(field_center_x, field_center_y - Units.inchesToMeters(127.375), shallow_height);
            cage_positions[1] = new Translation3d(field_center_x, field_center_y - Units.inchesToMeters(84.375), shallow_height);
            cage_positions[2] = new Translation3d(field_center_x, field_center_y - Units.inchesToMeters(41.5),shallow_height);
            cage_positions[3] = new Translation3d(field_center_x, field_center_y + Units.inchesToMeters(41.5), shallow_height);
            cage_positions[4] = new Translation3d(field_center_x, field_center_y + Units.inchesToMeters(84.375), shallow_height);
            cage_positions[5] = new Translation3d(field_center_x, field_center_y + Units.inchesToMeters(127.375), shallow_height);            
        }

        //i feel like this doesn't matter because who pushes code in that short of time unless we have a preconfig set up in smart dashboard - RN
        public static void setDeep(int location, boolean isDeep){
            Translation3d cage = cage_positions[location];

            if(isDeep)
                cage_positions[location] = new Translation3d(cage.getX(), cage.getY(), deep_height);
            else
                cage_positions[location] = new Translation3d(cage.getX(), cage.getY(), shallow_height);
        }
    }

    //i hate hexagons - RN
    public class Reef{
        
        public static double center_x = field_center_x - Units.inchesToMeters(168.692);
        public static double center_y = field_center_y/2;
        public static double from_reef = Units.inchesToMeters(65.5 / 2); 
        public static double long_radius = Units.inchesToMeters(75.238 / 2); 
        public static double side_length = Units.inchesToMeters(37.043);

        public static double branch_distance = Units.inchesToMeters(13);
        public static double branch_inset = Units.inchesToMeters(1.625);
        public static double top_inset = Units.inchesToMeters(1.125);

        public static double L1_front_height = Units.inchesToMeters(18);
        public static double L2_highest_h = Units.inchesToMeters(31.875);
        public static double L3_highest_h = Units.inchesToMeters(47.625);
        public static double L4_highest_h = Units.inchesToMeters(72.00);

        public class Base{
            
            public static Translation2d center = new Translation2d(center_x, center_y);

            //corners
            public static Translation2d left_src_corner = new Translation2d(
                center_x + long_radius * Math.cos(Units.degreesToRadians(150)) , 
                center_y + long_radius * Math.sin(Units.degreesToRadians(150)));
            public static Translation2d left_field_corner = new Translation2d(
                center_x + long_radius * Math.cos(Units.degreesToRadians(90)) , 
                center_y + long_radius * Math.sin(Units.degreesToRadians(90)));
                
            public static Translation2d left_brg_corner = new Translation2d(
                center_x + long_radius * Math.cos(Units.degreesToRadians(30)) , 
                center_y + long_radius * Math.sin(Units.degreesToRadians(30)));

            public static Translation2d right_brg_corner = new Translation2d(
                center_x + long_radius * Math.cos(Units.degreesToRadians(-30)) , 
                center_y + long_radius * Math.sin(Units.degreesToRadians(-30)));

            public static Translation2d right_field_corner = new Translation2d(
                center_x + long_radius * Math.cos(Units.degreesToRadians(-90)) , 
                center_y + long_radius * Math.sin(Units.degreesToRadians(-90)));

            public static Translation2d right_src_corner = new Translation2d(
                center_x + long_radius * Math.cos(Units.degreesToRadians(-150)) , 
                center_y + long_radius * Math.sin(Units.degreesToRadians(-150)));

            //walls
            public static Pose2d alliance_wall = new  Pose2d(
                center_x + from_reef * Math.cos(Units.degreesToRadians(180)) , 
                center_y + from_reef * Math.sin(Units.degreesToRadians(180)), 
                new Rotation2d(Degree.of(180.0)));

            public static Pose2d left_src_wall = new  Pose2d(
                center_x + from_reef * Math.cos(Units.degreesToRadians(120)) , 
                center_y + from_reef * Math.sin(Units.degreesToRadians(120)), 
                new Rotation2d(Degree.of(120.0)));
                
            public static  Pose2d left_brg_wall = new  Pose2d(
                center_x + from_reef * Math.cos(Units.degreesToRadians(60)) , 
                center_y + from_reef * Math.sin(Units.degreesToRadians(60)), 
                new Rotation2d(Degree.of(60.0)));

            public static Pose2d mid_brg_wall = new Pose2d(
                center_x + from_reef * Math.cos(Units.degreesToRadians(0)) , 
                center_y + from_reef * Math.sin(Units.degreesToRadians(0)), 
                new Rotation2d(Degree.of(0)));

            public static  Pose2d right_brg_wall = new Pose2d(
                center_x + from_reef * Math.cos(Units.degreesToRadians(-60)) , 
                center_y + from_reef * Math.sin(Units.degreesToRadians(-60)), 
                new Rotation2d(Degree.of(-60)));

            public static Pose2d right_src_wall = new  Pose2d(
                center_x + from_reef * Math.cos(Units.degreesToRadians(-120)) , 
                center_y + from_reef * Math.sin(Units.degreesToRadians(-120)), 
                new Rotation2d(Degree.of(-120)));
            
            
        }

        //crying - RN
        //based off cad which has algae radius of 8 which iiiissss within the tolerance but oh well - RN
        public class Algae{

            public static double from_reef = Units.inchesToMeters(26.746);
            public static double high_height = Units.inchesToMeters(51.575);
            public static double low_height = Units.inchesToMeters(35.6);

            public static Translation3d alliance_algae = new Translation3d(
                center_x + from_reef * Math.cos(Units.degreesToRadians(180)) , 
                center_y + from_reef * Math.sin(Units.degreesToRadians(180)), 
                high_height);

            public static Translation3d left_src_algae = new Translation3d(
                center_x + from_reef * Math.cos(Units.degreesToRadians(120)) , 
                center_y + from_reef * Math.sin(Units.degreesToRadians(120)), 
                low_height);
                
            public static Translation3d left_brg_algae = new Translation3d(
                center_x + from_reef * Math.cos(Units.degreesToRadians(60)) , 
                center_y + from_reef * Math.sin(Units.degreesToRadians(60)), 
                low_height);

            public static Translation3d mid_brg_algae = new Translation3d(
                center_x + from_reef * Math.cos(Units.degreesToRadians(0)) , 
                center_y + from_reef * Math.sin(Units.degreesToRadians(0)), 
                low_height);

            public static Translation3d right_brg_algae = new Translation3d(
                center_x + from_reef * Math.cos(Units.degreesToRadians(-60)) , 
                center_y + from_reef * Math.sin(Units.degreesToRadians(-60)), 
                high_height);

            public static Translation3d right_src_algae = new Translation3d(
                center_x + from_reef * Math.cos(Units.degreesToRadians(-120)) , 
                center_y + from_reef * Math.sin(Units.degreesToRadians(-120)), 
                low_height);
        }
    }

    public class Elem_Locations{
        public static double distance_x = Units.inchesToMeters(48);
        public static double distance_apart = Units.inchesToMeters(72);
        public static Translation2d[] corals_locations = new Translation2d[3];
        public static Translation3d[] algae_locations = new Translation3d[3];

        //ordered in disatnce from scoring table, 0 closest 2 farthest
        static{
            for(int i = 0; i < 3; i ++){

                corals_locations[i] = new Translation2d(distance_x, field_center_y - distance_apart + i * distance_apart);
                algae_locations[i] = new Translation3d(corals_locations[i].getX(), corals_locations[i].getY(), coral_length + algae_radius);
            }
        }
                
    }

    public class Processor{
        public static double opening_width = Units.inchesToMeters(28);
        public static double opening_height = Units.inchesToMeters(20);
        public static double opening_from_ground = Units.inchesToMeters(7);

        public static Pose2d processor_wall = new Pose2d(
            field_center_x + Units.inchesToMeters(109.712),
            field_width, 
            new Rotation2d(Degree.of(90)));
    }

    public class Source{
        public static double opening = Units.inchesToMeters(25.5);

        public static double src_length = Units.inchesToMeters(79.268);

        public static Translation2d left_src_alliance = new Translation2d(Units.inchesToMeters(15.819), Units.inchesToMeters(35.809));
        public static Translation2d left_src_mid = new Translation2d(Units.inchesToMeters(31.863), Units.inchesToMeters(23.8725));
        public static Translation2d left_src_reef = new Translation2d(Units.inchesToMeters(47.457), Units.inchesToMeters(11.936));
        public static Rotation2d left_src_rotat = new Rotation2d(Degree.of(54.0112));

        public static Translation2d right_src_alliance = new Translation2d(left_src_alliance.getX(), AllianceFlip.flipY(left_src_alliance.getY()));
        public static Translation2d right_src_mid = new Translation2d(left_src_mid.getX(), AllianceFlip.flipY(left_src_mid.getY()));
        public static Translation2d right_src_reef = new Translation2d( left_src_reef.getX(), AllianceFlip.flipY( left_src_reef.getY()));
        public static Rotation2d top_source_rotat = new Rotation2d(Degree.of(-54.0112));      
    }

}
