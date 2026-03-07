package frc.robot.AutoMovements;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;

public class DriveToPose extends Command {
  private static final double MAX_SPEED = 3.0; 
  private static final double DRIVE_TOLERANCE = 0.5; 
  private static final double THETA_TOLERANCE = 5.0; 

  private static final double PHASE_TRANSITION_Y_TOLERANCE = 0.4; 
  private static final double PHASE_TRANSITION_THETA_TOLERANCE = 15.0; 

  private enum Phase { Y_AND_HEADING, ALL }

  private final SwerveSubsystem swerve;
  private final LocalizationSubsystem localization;
  private final Supplier<Pose2d> targetSupplier;
  private final PIDController xController = new PIDController(4.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(4.0, 0.0, 0.0);
  private final PIDController thetaController = new PIDController(4.0, 0.0, 0.1);

  private final boolean simultaneous;

  private Pose2d targetPose;
  private Phase phase;

  /** Phased driving: Y+heading first, then all axes. */
  public DriveToPose(SwerveSubsystem swerve, LocalizationSubsystem localization, Supplier<Pose2d> target) {
    this(swerve, localization, target, false);
  }

  /**
   * @param simultaneous If true, drives X, Y, and rotation all at once from the start.
   *                     If false, uses the default phased approach (Y+heading first, then all).
   */
  public DriveToPose(SwerveSubsystem swerve, LocalizationSubsystem localization, Supplier<Pose2d> target, boolean simultaneous) {
    this.swerve = swerve;
    this.localization = localization;
    this.targetSupplier = target;
    this.simultaneous = simultaneous;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    targetPose = targetSupplier.get();
    xController.reset();
    yController.reset();
    thetaController.reset();
    phase = simultaneous ? Phase.ALL : Phase.Y_AND_HEADING;

    SmartDashboard.putString("DriveToPose/Target",
        String.format("(%.2f, %.2f, %.1f deg)", targetPose.getX(), targetPose.getY(),
            targetPose.getRotation().getDegrees()));
  }

  @Override
  public void execute() {
    Pose2d current = localization.getPose();

    double ySpeed = yController.calculate(current.getY(), targetPose.getY());
    double thetaSpeed = thetaController.calculate(
        current.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaSpeed = MathUtil.clamp(thetaSpeed, -Math.PI * 2, Math.PI * 2);

    double xSpeed = 0.0;
    double yError = Math.abs(current.getY() - targetPose.getY());
    double thetaError = Math.abs(current.getRotation().minus(targetPose.getRotation()).getDegrees());

    // Phase 1: drive Y and heading, no X
    // Phase 2: once Y and heading are roughly close, correct all axes together
    if (phase == Phase.Y_AND_HEADING) {
      if (yError < PHASE_TRANSITION_Y_TOLERANCE && thetaError < PHASE_TRANSITION_THETA_TOLERANCE) {
        phase = Phase.ALL;
      }
    }

    if (phase == Phase.ALL) {
      xSpeed = xController.calculate(current.getX(), targetPose.getX());
    }

    // Clamp linear speed
    Translation2d linearVelocity = new Translation2d(xSpeed, ySpeed);
    double magnitude = linearVelocity.getNorm();
    if (magnitude > MAX_SPEED) {
      linearVelocity = linearVelocity.times(MAX_SPEED / magnitude);
    }

    // Send as field-relative speeds
    swerve.setFieldRelativeAutoSpeeds(new ChassisSpeeds(
        linearVelocity.getX(), linearVelocity.getY(), thetaSpeed));

    // Telemetry
    double driveError = current.getTranslation().getDistance(targetPose.getTranslation());
    SmartDashboard.putNumber("DriveToPose/DriveError", driveError);
    SmartDashboard.putNumber("DriveToPose/YError", yError);
    SmartDashboard.putNumber("DriveToPose/ThetaErrorDeg", thetaError);
    SmartDashboard.putString("DriveToPose/Phase", phase.name());
    SmartDashboard.putBoolean("DriveToPose/AtGoal", isAtGoal());
  }

  @Override
  public void end(boolean interrupted) {
    // Stop driving
    swerve.setFieldRelativeAutoSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return isAtGoal();
  }

  private boolean isAtGoal() {
    Pose2d current = localization.getPose();
    double driveError = current.getTranslation().getDistance(targetPose.getTranslation());
    double thetaError = Math.abs(current.getRotation().minus(targetPose.getRotation()).getDegrees());
    return driveError < DRIVE_TOLERANCE && thetaError < THETA_TOLERANCE;
  }

  public boolean isInAllPhase() {
    return phase == Phase.ALL;
  }

  public double getDistanceToTarget() {
    if (targetPose == null) return Double.MAX_VALUE;
    return localization.getPose().getTranslation().getDistance(targetPose.getTranslation());

  }
}