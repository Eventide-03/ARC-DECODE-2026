package frc.robot.AutoMovements;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.PointWheelsAt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.fms.FmsSubsystem;
import frc.robot.FlywheelSubsystem.LookupTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.limelight.LimelightHelpers;


public class HeadingLock extends StateMachine<HeadingLock.HeadingLockState> {
  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private LookupTable lookupTable; 

  private Translation2d redTargetPoint = new Translation2d();
  private Translation2d blueTargetPoint = new Translation2d();
  private double redTurretOffsetDegrees = 315;
  private double blueTurretOffsetDegrees = -38;

  private static final String LIMELIGHT_LEFT = "limelight-left";
  private static final String LIMELIGHT_RIGHT = "limelight-right";
  private static final double STATIONARY_SPEED_THRESHOLD = 0.2; // m/s -- linear speed below this = stationary
  private static final double STATIONARY_ROT_THRESHOLD = 0.2; // rad/s -- rotational speed below this = not spinning
  private static final long MT1_CORRECTION_COOLDOWN_MS = 500;
  private long lastMt1CorrectionTimestamp = 0;
  private static final double MT1_MAX_CORRECTION_DEG = 15.0; // reject if diff > this (bad data)
  private static final double MT1_MIN_CORRECTION_DEG = 1.0;  // ignore if diff < this (not worth correcting)
  private static final double MT1_MIN_TAG_AREA = 0;       // reject if avg tag area too small (too far away)
  private boolean mt1HeadingCorrectionEnabled = true;
  private boolean mt1RightHeadingCorrectionEnabled = false;
  private static final double HEADING_TOLERANCE_DEG = 4.5;
  private double lastTargetAngleDeg = 0.0;
  private static final double HEADING_SETTLE_TIME_S = 0;
  private double headingOnTargetStartTime = 0;

 
  private static final double CLOSE_DISTANCE_M = 1.0;
  private static final double FAR_DISTANCE_M = 5;
  private static final double MAX_CORRECTION_DEG = 6;

  public enum HeadingLockState {
    DISABLED,
    RED_LOCK,
    BLUE_LOCK;
  }



  public HeadingLock(LocalizationSubsystem localization, SwerveSubsystem swerve) {
    super(SubsystemPriority.SWERVE, HeadingLockState.DISABLED);
    this.localization = localization;
    this.swerve = swerve;
  }

  public void setLookupTable(LookupTable table) { this.lookupTable = table; }

  public void setRedTargetPoint(Translation2d point) {
    this.redTargetPoint = point;
  }

  public void setBlueTargetPoint(Translation2d point) {
    this.blueTargetPoint = point;
  }



  public void setRedTargetPose(Pose2d pose) {
    this.redTargetPoint = pose.getTranslation();
  }

  public void setBlueTargetPose(Pose2d pose) {
    this.blueTargetPoint = pose.getTranslation();
  }

  public Pose2d getRedTargetPose() {
    return new Pose2d(redTargetPoint, Rotation2d.kZero);
  }

  public Pose2d getBlueTargetPose() {
    return new Pose2d(blueTargetPoint, Rotation2d.kZero);
  }

  public Translation2d getRedTargetPoint() {
    return redTargetPoint;
  }

  public Translation2d getBlueTargetPoint() {
    return blueTargetPoint;
  }

  public void setRedTurretOffsetDegrees(double offsetDegrees) {
    this.redTurretOffsetDegrees = offsetDegrees;
  }

  public void setBlueTurretOffsetDegrees(double offsetDegrees) {
    this.blueTurretOffsetDegrees = offsetDegrees;
  }

  public void setTurretOffsetDegrees(double offsetDegrees) {
    this.redTurretOffsetDegrees = offsetDegrees;
    this.blueTurretOffsetDegrees = offsetDegrees;
  }

  public void enableForAlliance() {
    if (FmsSubsystem.isRedAlliance()) {
      enableRedLock();
    } else {
      enableBlueLock();
    }
  }

  public void enableRedLock() {
    setStateFromRequest(HeadingLockState.RED_LOCK);
  }

  public void enableBlueLock() {
    setStateFromRequest(HeadingLockState.BLUE_LOCK);
  }

  public void disableLock() {
    setStateFromRequest(HeadingLockState.DISABLED);
    swerve.normalDriveRequest();
  }

  @Override
  protected HeadingLockState getNextState(HeadingLockState current) { return current; }

  @Override
  protected void collectInputs() {
    var robotTranslation = localization.getPose().getTranslation();
    SmartDashboard.putNumber("HeadingLock/DistToRed_m",
        robotTranslation.getDistance(redTargetPoint));
    SmartDashboard.putNumber("HeadingLock/DistToBlue_m",
        robotTranslation.getDistance(blueTargetPoint));
    checkAndCorrectHeadingWithMT1();
    checkAndCorrectHeadingWithMT1Right();
    switch (getState()) {
      case RED_LOCK -> faceTargetPoseBased(redTargetPoint, redTurretOffsetDegrees);
      case BLUE_LOCK -> faceTargetPoseBased(blueTargetPoint, blueTurretOffsetDegrees);
      case DISABLED -> {}
    }
  }


  private void checkAndCorrectHeadingWithMT1() {
    // Read toggle from dashboard
    mt1HeadingCorrectionEnabled = SmartDashboard.getBoolean("HeadingLock/MT1_Enabled", mt1HeadingCorrectionEnabled);
    SmartDashboard.putBoolean("HeadingLock/MT1_Enabled", mt1HeadingCorrectionEnabled);

    if (!mt1HeadingCorrectionEnabled) {
      SmartDashboard.putBoolean("HeadingLock/MT1_Correcting", false);
      SmartDashboard.putString("HeadingLock/MT1_Status", "DISABLED");
      return;
    }

    // Cooldown: don't correct more often than every 500ms
    long now = System.currentTimeMillis();
    if (now - lastMt1CorrectionTimestamp < MT1_CORRECTION_COOLDOWN_MS) {
      return;
    }

    // Only correct when robot is stationary (both linear and rotational)
    var speeds = swerve.getRobotRelativeSpeeds();
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    if (linearSpeed > STATIONARY_SPEED_THRESHOLD) {
      SmartDashboard.putBoolean("HeadingLock/MT1_Correcting", false);
      SmartDashboard.putString("HeadingLock/MT1_Status", "MOVING");
      return;
    }
    if (Math.abs(speeds.omegaRadiansPerSecond) > STATIONARY_ROT_THRESHOLD) {
      SmartDashboard.putBoolean("HeadingLock/MT1_Correcting", false);
      SmartDashboard.putString("HeadingLock/MT1_Status", "ROTATING");
      return;
    }

    var mt1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_LEFT);

    if (mt1Estimate == null || mt1Estimate.tagCount < 2) {
      SmartDashboard.putBoolean("HeadingLock/MT1_Correcting", false);
      SmartDashboard.putString("HeadingLock/MT1_Status", mt1Estimate == null ? "NO_DATA" : "NEED_2_TAGS");
      return;
    }

    if (mt1Estimate.avgTagArea < MT1_MIN_TAG_AREA) {
      SmartDashboard.putBoolean("HeadingLock/MT1_Correcting", false);
      SmartDashboard.putString("HeadingLock/MT1_Status", "TAGS_TOO_FAR");
      return;
    }

    double mt1HeadingDeg = mt1Estimate.pose.getRotation().getDegrees();
    double ourHeadingDeg = localization.getPose().getRotation().getDegrees();
    double diff = mt1HeadingDeg - ourHeadingDeg;
    // Normalize to [-180, 180]
    diff = ((diff + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    double absDiff = Math.abs(diff);

    SmartDashboard.putNumber("HeadingLock/MT1_HeadingDeg", mt1HeadingDeg);
    SmartDashboard.putNumber("HeadingLock/OurHeadingDeg", ourHeadingDeg);
    SmartDashboard.putNumber("HeadingLock/HeadingDiffDeg", diff);
    SmartDashboard.putNumber("HeadingLock/MT1_AvgTagArea", mt1Estimate.avgTagArea);

    // Reject if difference is too large (likely bad vision data)
    if (absDiff > MT1_MAX_CORRECTION_DEG) {
      SmartDashboard.putBoolean("HeadingLock/MT1_Correcting", false);
      SmartDashboard.putString("HeadingLock/MT1_Status", "DIFF_TOO_LARGE");
      return;
    }

    // Skip if difference is too small (not worth correcting)
    if (absDiff < MT1_MIN_CORRECTION_DEG) {
      SmartDashboard.putBoolean("HeadingLock/MT1_Correcting", false);
      SmartDashboard.putString("HeadingLock/MT1_Status", "DIFF_TOO_SMALL");
      return;
    }

    SmartDashboard.putBoolean("HeadingLock/MT1_Correcting", true);
    SmartDashboard.putString("HeadingLock/MT1_Status", "CORRECTING");
    localization.resetGyro(Rotation2d.fromDegrees(mt1HeadingDeg));
    lastMt1CorrectionTimestamp = now;
  }

  private void checkAndCorrectHeadingWithMT1Right() {
    // Read toggle from dashboard
    mt1RightHeadingCorrectionEnabled = SmartDashboard.getBoolean("HeadingLock/MT1_Right_Enabled", mt1RightHeadingCorrectionEnabled);
    SmartDashboard.putBoolean("HeadingLock/MT1_Right_Enabled", mt1RightHeadingCorrectionEnabled);

    if (!mt1RightHeadingCorrectionEnabled) {
      SmartDashboard.putBoolean("HeadingLock/MT1_Right_Correcting", false);
      SmartDashboard.putString("HeadingLock/MT1_Right_Status", "DISABLED");
      return;
    }

    // Cooldown: don't correct more often than every 500ms
    long now = System.currentTimeMillis();
    if (now - lastMt1CorrectionTimestamp < MT1_CORRECTION_COOLDOWN_MS) {
      return;
    }

    // Only correct when robot is stationary (both linear and rotational)
    var speeds = swerve.getRobotRelativeSpeeds();
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    if (linearSpeed > STATIONARY_SPEED_THRESHOLD) {
      SmartDashboard.putBoolean("HeadingLock/MT1_Right_Correcting", false);
      SmartDashboard.putString("HeadingLock/MT1_Right_Status", "MOVING");
      return;
    }
    if (Math.abs(speeds.omegaRadiansPerSecond) > STATIONARY_ROT_THRESHOLD) {
      SmartDashboard.putBoolean("HeadingLock/MT1_Right_Correcting", false);
      SmartDashboard.putString("HeadingLock/MT1_Right_Status", "ROTATING");
      return;
    }

    var mt1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_RIGHT);

    if (mt1Estimate == null || mt1Estimate.tagCount < 2) {
      SmartDashboard.putBoolean("HeadingLock/MT1_Right_Correcting", false);
      SmartDashboard.putString("HeadingLock/MT1_Right_Status", mt1Estimate == null ? "NO_DATA" : "NEED_2_TAGS");
      return;
    }

    if (mt1Estimate.avgTagArea < MT1_MIN_TAG_AREA) {
      SmartDashboard.putBoolean("HeadingLock/MT1_Right_Correcting", false);
      SmartDashboard.putString("HeadingLock/MT1_Right_Status", "TAGS_TOO_FAR");
      return;
    }

    double mt1HeadingDeg = mt1Estimate.pose.getRotation().getDegrees();
    double ourHeadingDeg = localization.getPose().getRotation().getDegrees();
    double diff = mt1HeadingDeg - ourHeadingDeg;
    // Normalize to [-180, 180]
    diff = ((diff + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    double absDiff = Math.abs(diff);

    SmartDashboard.putNumber("HeadingLock/MT1_Right_HeadingDeg", mt1HeadingDeg);
    SmartDashboard.putNumber("HeadingLock/MT1_Right_OurHeadingDeg", ourHeadingDeg);
    SmartDashboard.putNumber("HeadingLock/MT1_Right_HeadingDiffDeg", diff);
    SmartDashboard.putNumber("HeadingLock/MT1_Right_AvgTagArea", mt1Estimate.avgTagArea);

    // Reject if difference is too large (likely bad vision data)
    if (absDiff > MT1_MAX_CORRECTION_DEG) {
      SmartDashboard.putBoolean("HeadingLock/MT1_Right_Correcting", false);
      SmartDashboard.putString("HeadingLock/MT1_Right_Status", "DIFF_TOO_LARGE");
      return;
    }

    // Skip if difference is too small (not worth correcting)
    if (absDiff < MT1_MIN_CORRECTION_DEG) {
      SmartDashboard.putBoolean("HeadingLock/MT1_Right_Correcting", false);
      SmartDashboard.putString("HeadingLock/MT1_Right_Status", "DIFF_TOO_SMALL");
      return;
    }

    SmartDashboard.putBoolean("HeadingLock/MT1_Right_Correcting", true);
    SmartDashboard.putString("HeadingLock/MT1_Right_Status", "CORRECTING");
    localization.resetGyro(Rotation2d.fromDegrees(mt1HeadingDeg));
    lastMt1CorrectionTimestamp = now;
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
  }

  private void faceTargetPoseBased(Translation2d targetPoint, double offsetDegrees) {
    var robotPose = localization.getPose();
    double dx = targetPoint.getX() - robotPose.getX();
    double dy = targetPoint.getY() - robotPose.getY();

    double distance = Math.hypot(dx, dy);
    if (distance < 1e-6) {
      return;
    }

    // Add a small correction toward 180° as distance increases (max ~3 deg)
    double t = Math.max(0.0, Math.min(1.0, (distance - CLOSE_DISTANCE_M) / (FAR_DISTANCE_M - CLOSE_DISTANCE_M)));
    double correctionDeg = t * MAX_CORRECTION_DEG;

    double angleDegrees = Math.toDegrees(Math.atan2(dy, dx));
    double rawFinalAngle = angleDegrees + offsetDegrees;

    // Determine which direction to nudge toward 180°
    // Normalize to [-180, 180] to figure out the shortest path to 180
    double diff = 180.0 - rawFinalAngle;
    // Normalize diff to [-180, 180]
    diff = ((diff + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    double sign = Math.signum(diff);

    double finalAngle = rawFinalAngle + sign * correctionDeg;

    SmartDashboard.putNumber("HeadingLock/TargetAngleDeg", finalAngle);
    SmartDashboard.putNumber("HeadingLock/OffsetDeg", offsetDegrees);
    SmartDashboard.putNumber("HeadingLock/CorrectionDeg", sign * correctionDeg);
    SmartDashboard.putNumber("HeadingLock/DistanceM", distance);

    lastTargetAngleDeg = finalAngle;
    swerve.snapsDriveRequest(finalAngle);
  }

  /** Returns true if the robot heading is within tolerance of the heading lock target. */
  public boolean isOnTarget() {
    if (getState() == HeadingLockState.DISABLED) return false;
    double currentDeg = localization.getPose().getRotation().getDegrees();
    double error = lastTargetAngleDeg - currentDeg;
    // Normalize to [-180, 180]
    error = ((error + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    boolean onTarget = Math.abs(error) <= HEADING_TOLERANCE_DEG;
    SmartDashboard.putBoolean("HeadingLock/OnTarget", onTarget);
    SmartDashboard.putNumber("HeadingLock/HeadingErrorDeg", error);

    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    if (onTarget) {
      if (headingOnTargetStartTime < 0) {
        headingOnTargetStartTime = now;
      }
    } else {
      headingOnTargetStartTime = -1.0;
    }

    return onTarget;
  }

  /** Returns true if heading has been on target continuously for the settle time (1s). */
  public boolean isSettled() {
    boolean onTarget = isOnTarget();
    if (!onTarget || headingOnTargetStartTime < 0) {
      SmartDashboard.putBoolean("HeadingLock/Settled", false);
      return false;
    }
    double elapsed = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - headingOnTargetStartTime;
    boolean settled = elapsed >= HEADING_SETTLE_TIME_S;
    SmartDashboard.putBoolean("HeadingLock/Settled", settled);
    SmartDashboard.putNumber("HeadingLock/SettleElapsedS", elapsed);
    return settled;
  }

}
