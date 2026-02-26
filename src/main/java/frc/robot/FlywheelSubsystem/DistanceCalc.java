package frc.robot.FlywheelSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.AutoMovements.HeadingLock;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;


public class DistanceCalc extends LifecycleSubsystem {
  private final LocalizationSubsystem localization;
  private final HeadingLock headingLock;

  public DistanceCalc(LocalizationSubsystem localization, HeadingLock headingLock) {
    super(SubsystemPriority.LOCALIZATION);
    this.localization = localization;
    this.headingLock = headingLock;
  }


  public double getDistanceToAllianceTargetMeters() {
    Pose2d current = localization.getPose();
    Pose2d target = FmsSubsystem.isRedAlliance() ? headingLock.getRedTargetPose() : headingLock.getBlueTargetPose();
    return current.getTranslation().getDistance(target.getTranslation());
  }


  public double getRobotVelocityTowardTargetMetersPerSec() {
    Pose2d current = localization.getPose();
    Pose2d target = FmsSubsystem.isRedAlliance() ? headingLock.getRedTargetPose() : headingLock.getBlueTargetPose();

    double dx = target.getX() - current.getX();
    double dy = target.getY() - current.getY();
    double norm = Math.hypot(dx, dy);
    if (norm < 1e-6) {
      return 0.0;
    }

    ChassisSpeeds fieldSpeeds = localization.getFieldRelativeSpeeds();
    double vx = fieldSpeeds.vxMetersPerSecond;
    double vy = fieldSpeeds.vyMetersPerSecond;

    double ux = dx / norm;
    double uy = dy / norm;

    return vx * ux + vy * uy;
  }


  public double getRobotLateralVelocityMetersPerSec() {
    Pose2d current = localization.getPose();
    Pose2d target = FmsSubsystem.isRedAlliance() ? headingLock.getRedTargetPose() : headingLock.getBlueTargetPose();

    double dx = target.getX() - current.getX();
    double dy = target.getY() - current.getY();
    double norm = Math.hypot(dx, dy);
    if (norm < 1e-6) {
      return 0.0;
    }

    ChassisSpeeds fieldSpeeds = localization.getFieldRelativeSpeeds();
    double vx = fieldSpeeds.vxMetersPerSecond;
    double vy = fieldSpeeds.vyMetersPerSecond;

    double ux = dx / norm;
    double uy = dy / norm;
    double px = -uy;
    double py = ux;

    return vx * px + vy * py;
  }


  public double getRobotAngularVelocityRadPerSec() {
    return localization.getFieldRelativeSpeeds().omegaRadiansPerSecond;
  }

// look ahead exposure
  public edu.wpi.first.math.geometry.Pose2d getRobotPose() {
    return localization.getPose();
  }

  public double getFieldVelocityX() {
    return localization.getFieldRelativeSpeeds().vxMetersPerSecond;
  }

  public double getFieldVelocityY() {
    return localization.getFieldRelativeSpeeds().vyMetersPerSecond;
  }

  public edu.wpi.first.math.geometry.Pose2d getTargetPose() {
    return FmsSubsystem.isRedAlliance() ? headingLock.getRedTargetPose() : headingLock.getBlueTargetPose();
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
  }
}
