package frc.robot.AutoMovements;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.fms.FmsSubsystem;
import frc.robot.FlywheelSubsystem.LookupTable;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;


public class HeadingLock extends StateMachine<HeadingLock.HeadingLockState> {
  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private LookupTable lookupTable; 

  private Translation2d redTargetPoint = new Translation2d();
  private Translation2d blueTargetPoint = new Translation2d();
  private double turretOffsetDegrees = -40.765;  //-38.7

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

  public void setTurretOffsetDegrees(double offsetDegrees) {
    this.turretOffsetDegrees = offsetDegrees;
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
  }

  @Override
  protected HeadingLockState getNextState(HeadingLockState current) { return current; }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    switch (getState()) {
      case RED_LOCK -> faceTarget(redTargetPoint);
      case BLUE_LOCK -> faceTarget(blueTargetPoint);
      case DISABLED -> {
      }
    }
  }

  private void faceTarget(Translation2d targetPoint) {
    var robotPose = localization.getPose();
    double dx = targetPoint.getX() - robotPose.getX();
    double dy = targetPoint.getY() - robotPose.getY();

    double norm = Math.hypot(dx, dy);
    if (norm < 1e-6) {
      return;
    }

    // Use LookupTable's dynamic TOF (single source of truth)
    double tofSeconds = (lookupTable != null) ? lookupTable.getTimeOfFlightSeconds(norm) : 1.0;

    // Raw field-relative velocities
    ChassisSpeeds fieldSpeeds = localization.getFieldRelativeSpeeds();
    double vx = fieldSpeeds.vxMetersPerSecond;
    double vy = fieldSpeeds.vyMetersPerSecond;

    // Compute lookahead translation (pose-level lookahead)
    var lookahead = robotPose.getTranslation().plus(new Translation2d(vx * tofSeconds, vy * tofSeconds));

    double dxComp = targetPoint.getX() - lookahead.getX();
    double dyComp = targetPoint.getY() - lookahead.getY();

    double angleDegrees = Math.toDegrees(Math.atan2(dyComp, dxComp));
    swerve.snapsDriveRequest(angleDegrees + turretOffsetDegrees);
}

}
