package frc.robot.util;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class FieldConstant {
  public static double field_length = Units.inchesToMeters(690.875);
  public static double field_width = Units.inchesToMeters(317);
  public static double field_center_x = field_length / 2;
  public static double field_center_y = field_width / 2;

  public static Pose2d start_mid = new Pose2d(7.12, 3.5 + 0.53, Rotation2d.k180deg);
  public static Pose2d start_left = new Pose2d(7.12, 7.53, Rotation2d.k180deg);
  public static Pose2d start_right = new Pose2d(7.12, 0.53, Rotation2d.k180deg);

  public class Field {
    public static double brg_length = Units.inchesToMeters(23);
    public static double brg_width = Units.inchesToMeters(23);
    public static Translation2d brg_mid = new Translation2d(field_center_x, field_center_y);
    public static Translation2d alliance_right_corner =
        new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(50.750));
    public static Translation2d all_wall_right_corner =
        new Translation2d(Units.inchesToMeters(67.039), Units.inchesToMeters(0));
    public static Translation2d alliance_left_corner =
        new Translation2d(Units.inchesToMeters(0), field_width - Units.inchesToMeters(50.750));
    public static Translation2d all_wall_left_corner =
        new Translation2d(Units.inchesToMeters(67.039), field_width);

    public static Translation2d mid_left_wall = new Translation2d(field_center_x + 1, field_width);
    public static Translation2d mid_right_wall = new Translation2d(field_center_x + 1, 0);
  }

  public class Net {
    public static final double lineX = 8.027;
    public static final double lineYLow = 4.75;
    public static final double lineYHigh = 7.65;
    public static Pose2d left_score =
        new Pose2d(lineX, field_center_y + Units.inchesToMeters(127.375), Rotation2d.k180deg);
    public static Pose2d mid_score =
        new Pose2d(lineX, field_center_y + Units.inchesToMeters(84.375), Rotation2d.k180deg);
    public static Pose2d right_score =
        new Pose2d(lineX, field_center_y + Units.inchesToMeters(41.5), Rotation2d.k180deg);

    public static List<Pose2d> nscores =
        new LinkedList<Pose2d>(Arrays.asList(left_score, mid_score, right_score));

    public static void reloadNscores() {
      nscores = new LinkedList<Pose2d>(Arrays.asList(left_score, mid_score, right_score));
    }

    public static Pose2d getClosest(Pose2d current) {
      Pose2d unflipped = AllianceFlip.flipDS(current);
      return AllianceFlip.flipDS(
          new Pose2d(
              lineX, MathUtil.clamp(unflipped.getY(), lineYLow, lineYHigh), Rotation2d.k180deg));
    }
  }

  public class Reef {
    public static double center_x = field_center_x - Units.inchesToMeters(168.692);
    public static double center_y = field_width / 2;
    public static Translation2d center = new Translation2d(center_x, center_y);

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

    public class CoralGoal {
      public static double score_perp = Constants.robot.B_CROSSLENGTH / 2;
      public static double score_parallel = Units.inchesToMeters(6.472);

      public static Translation2d score_left = new Translation2d(-score_perp, score_parallel);
      public static Translation2d score_right = new Translation2d(-score_perp, -score_parallel);

      public static Pose2d alliance_left =
          new Pose2d(
              Base.alliance_wall
                  .getTranslation()
                  .plus(score_left.rotateBy(new Rotation2d(Degrees.of(0)))),
              new Rotation2d(Degrees.of(0)));

      public static Pose2d alliance_right =
          new Pose2d(
              Base.alliance_wall
                  .getTranslation()
                  .plus(score_right.rotateBy(new Rotation2d(Degrees.of(0)))),
              new Rotation2d(Degrees.of(0)));

      public static Pose2d left_src_left =
          new Pose2d(
              Base.left_src_wall
                  .getTranslation()
                  .plus(score_left.rotateBy(new Rotation2d(Degrees.of(-60)))),
              new Rotation2d(Degrees.of(-60)));

      public static Pose2d left_src_right =
          new Pose2d(
              Base.left_src_wall
                  .getTranslation()
                  .plus(score_right.rotateBy(new Rotation2d(Degrees.of(-60)))),
              new Rotation2d(Degrees.of(-60)));

      public static Pose2d right_src_left =
          new Pose2d(
              Base.right_src_wall
                  .getTranslation()
                  .plus(score_left.rotateBy(new Rotation2d(Degrees.of(60)))),
              new Rotation2d(Degrees.of(60)));

      public static Pose2d right_src_right =
          new Pose2d(
              Base.right_src_wall
                  .getTranslation()
                  .plus(score_right.rotateBy(new Rotation2d(Degrees.of(60)))),
              new Rotation2d(Degrees.of(60)));

      public static Pose2d left_brg_left =
          new Pose2d(
              Base.left_brg_wall
                  .getTranslation()
                  .plus(score_left.rotateBy(new Rotation2d(Degrees.of(-120)))),
              new Rotation2d(Degrees.of(-120)));

      public static Pose2d left_brg_right =
          new Pose2d(
              Base.left_brg_wall
                  .getTranslation()
                  .plus(score_right.rotateBy(new Rotation2d(Degrees.of(-120)))),
              new Rotation2d(Degrees.of(-120)));

      public static Pose2d right_brg_left =
          new Pose2d(
              Base.right_brg_wall
                  .getTranslation()
                  .plus(score_left.rotateBy(new Rotation2d(Degrees.of(120)))),
              new Rotation2d(Degrees.of(120)));

      public static Pose2d right_brg_right =
          new Pose2d(
              Base.right_brg_wall
                  .getTranslation()
                  .plus(score_right.rotateBy(new Rotation2d(Degrees.of(120)))),
              new Rotation2d(Degrees.of(120)));

      public static Pose2d mid_brg_left =
          new Pose2d(
              Base.mid_brg_wall
                  .getTranslation()
                  .plus(score_left.rotateBy(new Rotation2d(Degrees.of(180)))),
              new Rotation2d(Degrees.of(180)));

      public static Pose2d mid_brg_right =
          new Pose2d(
              Base.mid_brg_wall
                  .getTranslation()
                  .plus(score_right.rotateBy(new Rotation2d(Degrees.of(180)))),
              new Rotation2d(Degrees.of(180)));

      public static List<Pose2d> cgoals =
          new LinkedList<Pose2d>(
              Arrays.asList(
                  alliance_right,
                  alliance_left,
                  left_brg_left,
                  left_brg_right,
                  left_src_left,
                  left_src_right,
                  right_brg_left,
                  right_brg_right,
                  right_src_left,
                  right_src_right,
                  mid_brg_left,
                  mid_brg_right));
    }

    public class AlgaeSource {
      public static double src_perp = Constants.robot.B_LENGTH / 2;
      public static double src_parallel = 0;

      public static Translation2d source = new Translation2d(-src_perp, src_parallel);

      public static Pose2d alliance_src =
          new Pose2d(
              Base.alliance_wall
                  .getTranslation()
                  .plus(source.rotateBy(new Rotation2d(Degrees.of(0)))),
              new Rotation2d(Degrees.of(0)));

      public static Pose2d left_brg_src =
          new Pose2d(
              Base.left_brg_wall
                  .getTranslation()
                  .plus(source.rotateBy(new Rotation2d(Degrees.of(-120)))),
              new Rotation2d(Degrees.of(-120)));

      public static Pose2d right_brg_src =
          new Pose2d(
              Base.right_brg_wall
                  .getTranslation()
                  .plus(source.rotateBy(new Rotation2d(Degrees.of(120)))),
              new Rotation2d(Degrees.of(120)));

      public static Pose2d left_src_src =
          new Pose2d(
              Base.left_src_wall
                  .getTranslation()
                  .plus(source.rotateBy(new Rotation2d(Degrees.of(-60)))),
              new Rotation2d(Degrees.of(-60)));

      public static Pose2d right_src_src =
          new Pose2d(
              Base.right_src_wall
                  .getTranslation()
                  .plus(source.rotateBy(new Rotation2d(Degrees.of(60)))),
              new Rotation2d(Degrees.of(60)));

      public static Pose2d mid_brg_src =
          new Pose2d(
              Base.mid_brg_wall
                  .getTranslation()
                  .plus(source.rotateBy(new Rotation2d(Degrees.of(180)))),
              new Rotation2d(Degrees.of(180)));

      public static List<Pose2d> asources =
          new LinkedList<Pose2d>(
              Arrays.asList(
                  alliance_src,
                  left_brg_src,
                  right_brg_src,
                  left_src_src,
                  right_src_src,
                  mid_brg_src));

      public static void reloadAsources() {
        asources =
            new LinkedList<Pose2d>(
                Arrays.asList(
                    alliance_src,
                    left_brg_src,
                    right_brg_src,
                    left_src_src,
                    right_src_src,
                    mid_brg_src));
      }

      public static int algaeHeight(Pose2d algae_src) {
        if (algae_src == alliance_src || algae_src == left_brg_src || algae_src == right_brg_src) {
          return 3;
        } else if (algae_src == left_src_src
            || algae_src == right_src_src
            || algae_src == mid_brg_src) {
          return 2;
        } else {
          return 0;
        }
      }
    }

    public class Base {

      public static Translation2d center = new Translation2d(center_x, center_y);

      public static Translation2d left_src_corner =
          new Translation2d(
              center_x + long_radius * Math.cos(Units.degreesToRadians(150)),
              center_y + long_radius * Math.sin(Units.degreesToRadians(150)));
      public static Translation2d left_field_corner =
          new Translation2d(
              center_x + long_radius * Math.cos(Units.degreesToRadians(90)),
              center_y + long_radius * Math.sin(Units.degreesToRadians(90)));

      public static Translation2d left_brg_corner =
          new Translation2d(
              center_x + long_radius * Math.cos(Units.degreesToRadians(30)),
              center_y + long_radius * Math.sin(Units.degreesToRadians(30)));

      public static Translation2d right_brg_corner =
          new Translation2d(
              center_x + long_radius * Math.cos(Units.degreesToRadians(-30)),
              center_y + long_radius * Math.sin(Units.degreesToRadians(-30)));

      public static Translation2d right_field_corner =
          new Translation2d(
              center_x + long_radius * Math.cos(Units.degreesToRadians(-90)),
              center_y + long_radius * Math.sin(Units.degreesToRadians(-90)));

      public static Translation2d right_src_corner =
          new Translation2d(
              center_x + long_radius * Math.cos(Units.degreesToRadians(-150)),
              center_y + long_radius * Math.sin(Units.degreesToRadians(-150)));

      public static Pose2d alliance_wall =
          new Pose2d(
              center_x + from_reef * Math.cos(Units.degreesToRadians(180)),
              center_y + from_reef * Math.sin(Units.degreesToRadians(180)),
              new Rotation2d(Degrees.of(180.0)));

      public static Pose2d left_src_wall =
          new Pose2d(
              center_x + from_reef * Math.cos(Units.degreesToRadians(120)),
              center_y + from_reef * Math.sin(Units.degreesToRadians(120)),
              new Rotation2d(Degrees.of(120.0)));

      public static Pose2d left_brg_wall =
          new Pose2d(
              center_x + from_reef * Math.cos(Units.degreesToRadians(60)),
              center_y + from_reef * Math.sin(Units.degreesToRadians(60)),
              new Rotation2d(Degrees.of(60.0)));

      public static Pose2d mid_brg_wall =
          new Pose2d(
              center_x + from_reef * Math.cos(Units.degreesToRadians(0)),
              center_y + from_reef * Math.sin(Units.degreesToRadians(0)),
              new Rotation2d(Degrees.of(0)));

      public static Pose2d right_brg_wall =
          new Pose2d(
              center_x + from_reef * Math.cos(Units.degreesToRadians(-60)),
              center_y + from_reef * Math.sin(Units.degreesToRadians(-60)),
              new Rotation2d(Degrees.of(-60)));

      public static Pose2d right_src_wall =
          new Pose2d(
              center_x + from_reef * Math.cos(Units.degreesToRadians(-120)),
              center_y + from_reef * Math.sin(Units.degreesToRadians(-120)),
              new Rotation2d(Degrees.of(-120)));
    }
  }

  public class Processor {
    public static Translation2d score_pos =
        new Translation2d(-Constants.robot.B_LENGTH / 2 - 0.1, 0.3);

    public static Pose2d processor_wall =
        new Pose2d(
            field_center_x - Units.inchesToMeters(109.712) - Units.inchesToMeters(20),
            0,
            new Rotation2d(Degree.of(90)));

    public static Pose2d processor_goal =
        new Pose2d(
            processor_wall.getTranslation().plus(score_pos.rotateBy(Rotation2d.fromDegrees(-90))),
            Rotation2d.fromDegrees(-90));
  }

  public class Source {
    public static Pose2d right_src_wall =
        new Pose2d(
            Units.inchesToMeters(31.863),
            Units.inchesToMeters(23.8725),
            Rotation2d.fromDegrees(54.0112));
    public static Pose2d left_src_wall =
        new Pose2d(
            Units.inchesToMeters(31.863),
            field_width - Units.inchesToMeters(23.8725),
            Rotation2d.fromDegrees(-54.0112));
    public static double src_length = Units.inchesToMeters(79.268);

    public static Translation2d src_pos =
        new Translation2d(-Constants.robot.B_CROSSLENGTH / 3 * 2, 0);

    public static Translation2d right_src_off =
        (new Translation2d(Units.inchesToMeters(8), 0))
            .rotateBy(Rotation2d.fromDegrees(54.0112 - 90));
    public static Translation2d left_src_off =
        (new Translation2d(Units.inchesToMeters(8), 0))
            .rotateBy(Rotation2d.fromDegrees(-54.0112 + 90));

    public static Pose2d right_src_close =
        new Pose2d(
            right_src_wall
                .getTranslation()
                .minus(right_src_off.times(3))
                .plus(src_pos.rotateBy(Rotation2d.fromDegrees(54.0112 - 180))),
            Rotation2d.fromDegrees(54.0112 - 180));

    public static Pose2d right_src_mid =
        new Pose2d(
            right_src_wall
                .getTranslation()
                .minus(right_src_off.times(0))
                .plus(src_pos.rotateBy(Rotation2d.fromDegrees(54.0112 - 180))),
            Rotation2d.fromDegrees(54.0112 - 180));

    public static Pose2d right_src_far =
        new Pose2d(
            right_src_wall
                .getTranslation()
                .plus(right_src_off.times(3))
                .plus(src_pos.rotateBy(Rotation2d.fromDegrees(54.0112 - 180))),
            Rotation2d.fromDegrees(54.0112 - 180));

    public static Pose2d left_src_close =
        new Pose2d(
            left_src_wall
                .getTranslation()
                .minus(left_src_off.times(3))
                .plus(src_pos.rotateBy(Rotation2d.fromDegrees(-54.0112 + 180))),
            Rotation2d.fromDegrees(-54.0112 + 180));

    public static Pose2d left_src_mid =
        new Pose2d(
            left_src_wall
                .getTranslation()
                .plus(left_src_off.times(0))
                .plus(src_pos.rotateBy(Rotation2d.fromDegrees(-54.0112 + 180))),
            Rotation2d.fromDegrees(-54.0112 + 180));

    public static Pose2d left_src_far =
        left_src_mid =
            new Pose2d(
                left_src_wall
                    .getTranslation()
                    .plus(left_src_off.times(3))
                    .plus(src_pos.rotateBy(Rotation2d.fromDegrees(-54.0112 + 180))),
                Rotation2d.fromDegrees(-54.0112 + 180));

    public static List<Pose2d> csources =
        List.of(
            right_src_close,
            right_src_mid,
            right_src_far,
            left_src_close,
            left_src_mid,
            left_src_far);
  }
}
