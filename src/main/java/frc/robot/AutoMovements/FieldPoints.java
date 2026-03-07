package frc.robot.AutoMovements;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


public final class FieldPoints {
  private FieldPoints() {}

  //  redneurtal zone poses
  private static Pose2d RA1 = new Pose2d(12.955, 7.409, Rotation2d.kZero);
  private static Pose2d RB1 = new Pose2d(7.409,7.409, Rotation2d.kZero);
 // private static Pose2d RC1 = new Pose2d(2.0, 0.0, Rotation2d.kZero);
  private static Pose2d RA2 = new Pose2d(12.955, 0.674, Rotation2d.kZero);
  private static Pose2d RB2 = new Pose2d(7.409, 0.674, Rotation2d.kZero);
 // private static Pose2d RC2 = new Pose2d(2.0, 1.0, Rotation2d.kZero);

  // blue neutral zone poses
  private static Pose2d BA1 = new Pose2d(3.591, 7.409, Rotation2d.kZero);
  private static Pose2d BB1 = new Pose2d(5.712, 7.409, Rotation2d.kZero);
//  private static Pose2d BC1 = new Pose2d(2.0, 0.0, Rotation2d.kZero);
  private static Pose2d BA2 = new Pose2d(3.591, 0.674, Rotation2d.kZero);
  private static Pose2d BB2 = new Pose2d(5.712, 0.674, Rotation2d.kZero);
 // private static Pose2d BC2 = new Pose2d(2.0, 1.0, Rotation2d.kZero);

  // points for heading lock
  private static Translation2d HEADINGLOCK_BLUE_POINT = new Translation2d(4.623818397521973, 4.034599781036377);
  private static Translation2d HEADINGLOCK_RED_POINT = new Translation2d(11.932568550109863, 4);
  
  // outpost poses
  private static Pose2d OUTPOST_BLUE= new Pose2d(0.766, 0.433, Rotation2d.fromDegrees(0));
  private static Pose2d OUTPOST_RED = new Pose2d(16.25, 7.291, Rotation2d.fromDegrees(180.0));


  // Trench zones (X and Y bounds)
  // Trench X bounds: only lock when robot X is between 10.8 and 13.0
  public static final double TRENCH_X_MIN = 10.4;
  public static final double TRENCH_X_MAX = 13.2;
  // Right trench (top of field): lock Y to 7.5 when robot Y > 6.8
  public static final double RIGHT_TRENCH_Y_THRESHOLD = 6.8;
  public static final double RIGHT_TRENCH_Y_LOCK = 7.5;
  // Left trench (bottom of field): lock Y to 0.6 when robot Y < 1.3
  public static final double LEFT_TRENCH_Y_THRESHOLD = 1.3;
  public static final double LEFT_TRENCH_Y_LOCK = 0.6;

  // Pass target points
  public static final Translation2d PASS_TARGET_RIGHT = new Translation2d(14.0, 7.0);
  public static final Translation2d PASS_TARGET_LEFT  = new Translation2d(14.0, 3.5);

  // Y approach zones — ramp assist in as driver approaches trench Y threshold
  // Right trench: approach starts at 5.5, full lock at 6.8
  public static final double RIGHT_TRENCH_APPROACH_Y = 5.5;
  // Left trench: approach starts at 2.5, full lock at 1.3
  public static final double LEFT_TRENCH_APPROACH_Y = 2.5;

  // getter stuff
  public static Pose2d getRA1() { return RA1; }
  public static Pose2d getRB1() { return RB1; }
  public static Pose2d getRA2() { return RA2; }
  public static Pose2d getRB2() { return RB2; }

  public static Pose2d getBA1() { return BA1; }
  public static Pose2d getBB1() { return BB1; }
  public static Pose2d getBA2() { return BA2; }
  public static Pose2d getBB2() { return BB2; }

  public static Translation2d getHeadingLockRedPoint() { return HEADINGLOCK_RED_POINT; }
  public static Translation2d getHeadingLockBluePoint() { return HEADINGLOCK_BLUE_POINT; }

  public static Pose2d getOutpostRed() { return OUTPOST_RED; }
  public static Pose2d getOutpostBlue() { return OUTPOST_BLUE; }

  // setter stuff
  public static void setRA1(Pose2d v) { RA1 = v; }
  public static void setRB1(Pose2d v) { RB1 = v; }
  public static void setRA2(Pose2d v) { RA2 = v; }
  public static void setRB2(Pose2d v) { RB2 = v; }

  public static void setBA1(Pose2d v) { BA1 = v; }
  public static void setBB1(Pose2d v) { BB1 = v; }
  public static void setBA2(Pose2d v) { BA2 = v; }
  public static void setBB2(Pose2d v) { BB2 = v; }

  public static void setHeadingLockRedPoint(Translation2d v) { HEADINGLOCK_RED_POINT = v; }
  public static void setHeadingLockBluePoint(Translation2d v) { HEADINGLOCK_BLUE_POINT = v; }

  public static void setOutpostRed(Pose2d v) { OUTPOST_RED = v; }
  public static void setOutpostBlue(Pose2d v) { OUTPOST_BLUE = v; }
}
