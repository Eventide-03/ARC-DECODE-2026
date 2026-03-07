package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AutoMovements.DriveToPose;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * A fluent builder for creating autonomous routines using DriveToPose point-to-point driving.
 *
 * Usage example:
 * <pre>
 *   Command auto = AutoRoutine.create(swerve, localization)
 *       .startAt(14.0, 7.0, 180.0)               // reset pose to starting position
 *       .driveTo(12.0, 7.0, 180.0)                // drive to first point
 *       .doWhileDriving(spinUpFlywheel())          // run a command in parallel with next drive
 *       .driveTo(11.0, 6.0, 200.0)                // drive while spinning up
 *       .run(shootCommand())                       // run a command (waits for it to finish)
 *       .driveTo(10.0, 5.0, 180.0)                // drive to next point
 *       .runFor(0.5, feedCommand())                // run a command for 0.5 seconds
 *       .waitSeconds(0.3)                          // pause
 *       .build();
 * </pre>
 */
public class AutoRoutine {
  private final SwerveSubsystem swerve;
  private final LocalizationSubsystem localization;
  private final List<Command> steps = new ArrayList<>();
  private Pose2d startPose = null;

  // Queued parallel commands to run alongside the NEXT driveTo
  private final List<Command> pendingParallel = new ArrayList<>();

  private AutoRoutine(SwerveSubsystem swerve, LocalizationSubsystem localization) {
    this.swerve = swerve;
    this.localization = localization;
  }

  /** Create a new auto routine builder. */
  public static AutoRoutine create(SwerveSubsystem swerve, LocalizationSubsystem localization) {
    return new AutoRoutine(swerve, localization);
  }

  // ---- Starting pose ----

  /** Set the starting pose. The robot's localization will be reset to this pose at auto start. */
  public AutoRoutine startAt(Pose2d pose) {
    this.startPose = pose;
    return this;
  }

  /** Set the starting pose from X, Y (meters) and heading (degrees). */
  public AutoRoutine startAt(double x, double y, double headingDeg) {
    return startAt(new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg)));
  }

  // ---- Drive steps ----

  /** Drive to a pose. Any pending parallel commands (from doWhileDriving) run alongside this. */
  public AutoRoutine driveTo(Pose2d target) {
    return driveTo(() -> target);
  }

  /** Drive to a pose from X, Y (meters) and heading (degrees). */
  public AutoRoutine driveTo(double x, double y, double headingDeg) {
    return driveTo(new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg)));
  }

  /** Drive to a dynamically supplied pose (evaluated at command start). */
  public AutoRoutine driveTo(Supplier<Pose2d> target) {
    return addDriveStep(target, false);
  }

  // ---- Simultaneous drive (X + Y + rotation all at once) ----

  /** Drive to a pose with X, Y, and rotation all moving simultaneously. */
  public AutoRoutine driveToAll(Pose2d target) {
    return driveToAll(() -> target);
  }

  /** Drive to a pose from X, Y (meters) and heading (degrees) with all axes simultaneously. */
  public AutoRoutine driveToAll(double x, double y, double headingDeg) {
    return driveToAll(new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg)));
  }

  /** Drive to a dynamically supplied pose with all axes simultaneously. */
  public AutoRoutine driveToAll(Supplier<Pose2d> target) {
    return addDriveStep(target, true);
  }

  /** Internal helper to add a drive step with or without simultaneous mode. */
  private AutoRoutine addDriveStep(Supplier<Pose2d> target, boolean simultaneous) {
    Command drive = new DriveToPose(swerve, localization, target, simultaneous);

    if (!pendingParallel.isEmpty()) {
      // Run all pending parallel commands alongside this drive
      List<Command> parallel = new ArrayList<>(pendingParallel);
      pendingParallel.clear();
      // The drive is the "main" command; parallels run alongside it
      Command combined = drive;
      for (Command cmd : parallel) {
        combined = combined.alongWith(cmd);
      }
      steps.add(combined.withName("DriveWithParallel"));
    } else {
      steps.add(drive);
    }
    return this;
  }

  // ---- Parallel commands (run alongside the NEXT driveTo) ----

  /** Queue a command to run in parallel with the NEXT driveTo call. Can be called multiple times. */
  public AutoRoutine doWhileDriving(Command cmd) {
    pendingParallel.add(cmd);
    return this;
  }

  // ---- Sequential action steps (run between drives) ----

  /** Run a command and wait for it to finish before continuing. */
  public AutoRoutine run(Command cmd) {
    steps.add(cmd);
    return this;
  }

  /** Run a command that fires instantly (runOnce). */
  public AutoRoutine runOnce(Runnable action) {
    steps.add(Commands.runOnce(action));
    return this;
  }

  /** Run a command for a fixed duration, then move on. */
  public AutoRoutine runFor(double seconds, Command cmd) {
    steps.add(cmd.withTimeout(seconds));
    return this;
  }

  /** Wait for a duration before continuing. */
  public AutoRoutine waitSeconds(double seconds) {
    steps.add(Commands.waitSeconds(seconds));
    return this;
  }

  /** Run a command until a condition is true. */
  public AutoRoutine runUntil(java.util.function.BooleanSupplier condition, Command cmd) {
    steps.add(cmd.until(condition));
    return this;
  }

  // ---- Build ----

  /** Build the auto routine into a single Command. */
  public Command build() {
    List<Command> allSteps = new ArrayList<>();

    // First step: reset pose if startAt was called
    if (startPose != null) {
      final Pose2d pose = startPose;
      allSteps.add(Commands.runOnce(() -> {
        localization.resetPose(pose);
        localization.resetGyro(pose.getRotation());
      }).withName("ResetPose"));
    }

    allSteps.addAll(steps);

    // Stop the robot at the end
    allSteps.add(Commands.runOnce(() -> {
      swerve.setFieldRelativeAutoSpeeds(new edu.wpi.first.math.kinematics.ChassisSpeeds());
    }).withName("StopDrive"));

    return Commands.sequence(allSteps.toArray(new Command[0])).withName("AutoRoutine");
  }
}
