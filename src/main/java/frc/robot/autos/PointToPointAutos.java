package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AutoMovements.HeadingLock;
import frc.robot.FlywheelSubsystem.Flywheel;
import frc.robot.FlywheelSubsystem.Hood;
import frc.robot.FlywheelSubsystem.LookupTable;
import frc.robot.FlywheelSubsystem.FlywheelStateMachine;
import frc.robot.FlywheelSubsystem.HoodStateMachine;
import frc.robot.IndexerSubsystem.Indexer;
import frc.robot.IndexerSubsystem.Hopper;
import frc.robot.Intake.IntakePosition;
import frc.robot.Intake.intaker;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;

/**
 * All point-to-point autonomous routines live here.
 *
 * ===== HOW TO ADD A NEW AUTO =====
 * 1. Add a new method below (copy an existing one as a template)
 * 2. Use AutoRoutine.create(swerve, localization) to start building
 * 3. Chain steps:
 *      .startAt(x, y, deg)            - set starting pose
 *      .driveTo(x, y, deg)            - drive to a field pose
 *      .runOnce(() -> ...)             - instant action
 *      .run(command)                   - run command, wait for it to finish
 *      .runFor(seconds, command)       - run command for N seconds
 *      .waitSeconds(seconds)           - pause
 *      .doWhileDriving(command)        - run in parallel with NEXT driveTo
 *      .build()                        - finalize
 * 4. Register in constructor: chooser.addOption("Name", myNewAuto())
 *
 * ===== HOW TO ADD A POINT TO AN EXISTING AUTO =====
 * Just insert .driveTo(x, y, heading) wherever you want in the chain.
 * Add .runOnce() / .doWhileDriving() around it for actions.
 */
public class PointToPointAutos {
  private static final double SHOOT_FEED_DELAY_SEC = 0.3;
  private static final double SHOOT_INTAKE_DUTY = 0.4;

  private final SwerveSubsystem swerve;
  private final LocalizationSubsystem localization;
  private final Flywheel flywheel;
  private final Hood hood;
  private final FlywheelStateMachine flywheelSM;
  private final HoodStateMachine hoodSM;
  private final HeadingLock headingLock;
  private final LookupTable turretLookup;
  private final Indexer indexer;
  private final Hopper hopper;
  private final intaker intakeRoller;
  private final IntakePosition intakePosition;

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  public PointToPointAutos(
      SwerveSubsystem swerve,
      LocalizationSubsystem localization,
      Flywheel flywheel,
      Hood hood,
      FlywheelStateMachine flywheelSM,
      HoodStateMachine hoodSM,
      HeadingLock headingLock,
      LookupTable turretLookup,
      Indexer indexer,
      Hopper hopper,
      intaker intakeRoller,
      IntakePosition intakePosition) {
    this.swerve = swerve;
    this.localization = localization;
    this.flywheel = flywheel;
    this.hood = hood;
    this.flywheelSM = flywheelSM;
    this.hoodSM = hoodSM;
    this.headingLock = headingLock;
    this.turretLookup = turretLookup;
    this.indexer = indexer;
    this.hopper = hopper;
    this.intakeRoller = intakeRoller;
    this.intakePosition = intakePosition;

    // ===== REGISTER ALL AUTOS HERE =====
    chooser.setDefaultOption("Do Nothing", Commands.none());
    chooser.addOption("Red Right", RedRight());
    chooser.addOption("Red Left", RedLeft());

    SmartDashboard.putData("Auto Chooser", chooser);
  }

  /** Get the currently selected auto command from the dashboard chooser. */
  public Command getSelected() {
    return chooser.getSelected();
  }

  // =====================================================================
  //  HELPER COMMANDS - reusable building blocks for any auto
  // =====================================================================

  private Command startAiming() {
    return Commands.runOnce(() -> {
      headingLock.enableForAlliance();
      turretLookup.enable();
    });
  }

  private Command stopAiming() {
    return Commands.runOnce(() -> {
      headingLock.disableLock();
      turretLookup.disable();
      flywheelSM.requestOff();
      hoodSM.requestOff();
    });
  }

  private Command startFeeding() {
    return Commands.waitUntil(() -> flywheel.isAtGoal() && headingLock.isSettled())
        .andThen(Commands.waitSeconds(SHOOT_FEED_DELAY_SEC))
        .andThen(Commands.runOnce(() -> {
          intakeRoller.setDutyPercent(SHOOT_INTAKE_DUTY);
          indexer.feed();
          hopper.feed();
        }))
        .withName("WaitThenFeed");
  }

  private Command stopFeeding() {
    return Commands.runOnce(() -> {
      intakeRoller.stop();
      indexer.stop();
      hopper.stop();
    });
  }

  private Command startIntaking() {
    return Commands.runOnce(() -> {
      intakePosition.deploy();
      intakeRoller.intake();
      hopper.feed();
    });
  }

  private Command stopIntaking() {
    return Commands.runOnce(() -> {
      intakePosition.retract();
      intakeRoller.stop();
      hopper.stop();
    });
  }

  private Command stopAll() {
    return Commands.runOnce(() -> {
      headingLock.disableLock();
      turretLookup.disable();
      flywheelSM.requestOff();
      hoodSM.requestOff();
      indexer.stop();
      hopper.stop();
      intakeRoller.stop();
      intakePosition.retract();
    });
  }


  private Command RedLeft() {
    return AutoRoutine.create(swerve, localization)
        .startAt(12.84, 0.7, 180.0)
        .driveToAll(8.8, 0.7, 270)
        .run(startIntaking())
        .driveToAll(8.8, 3.6, 270)
        .driveToAll(8.8, 0.7, 51.56)
        .run(stopIntaking())
        .driveToAll(15.3, 0.8, 180)
        .run(startAiming())
        .run(startFeeding())
        .waitSeconds(3)
        .run(stopAiming())
        .run(stopFeeding())
        .driveToAll(8.8, 0.7, 270)
        .run(startIntaking())
        .driveToAll(8.8, 3.6, 270)
        .driveToAll(8.8, 0.7, 180)
        .run(stopIntaking())
        .driveToAll(15.3, 0.7, 51.56)
        .run(startAiming())
        .run(startFeeding())
        .waitSeconds(3)
        .run(stopAll())
        .build()
        .withName("Red Left");
  }

  private Command RedRight() {
    return AutoRoutine.create(swerve, localization)
        .startAt(13.0, 7.5, 180.0)
        .driveToAll(9, 7.5, 90)
        .run(startIntaking())
        .driveToAll(9, 4.6, 90)
        .driveToAll(9,7.45, 180)
        .run(stopIntaking())
        .driveToAll(13.2, 7.5, 180)
        .driveToAll(15, 6.7,180)
        .run(startAiming())
        .run(startFeeding())
        .waitSeconds(3)
        .driveToAll(13, 7.5, 180)
        .driveToAll(9, 7.5, 90)
        .run(startIntaking())
        .driveToAll(9, 4.6, 90)
        .driveToAll(9,7.45, 180)
        .run(stopIntaking())
        .driveToAll(13.2, 7.5, 180)
        .driveToAll(16.25, 7.291,180)
        .run(startAiming())
        .run(startFeeding())
        .waitSeconds(5)
        .run(stopAll())
        .build()
        .withName("Red Right");
}



 
}
