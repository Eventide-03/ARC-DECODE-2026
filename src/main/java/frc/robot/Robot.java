package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.AutoMovements.HeadingLock;
import frc.robot.AutoMovements.OutpostSetpoint;
import frc.robot.FlywheelSubsystem.DistanceCalc;
import frc.robot.FlywheelSubsystem.LookupTable;
import frc.robot.Intake.IntakePosition;
import frc.robot.Intake.intaker;
import frc.robot.FlywheelSubsystem.Flywheel;
import frc.robot.FlywheelSubsystem.Hood;
import frc.robot.FlywheelSubsystem.FlywheelStateMachine;
import frc.robot.FlywheelSubsystem.HoodStateMachine;
import frc.robot.IndexerSubsystem.Indexer;
import frc.robot.IndexerSubsystem.Hopper;
import frc.robot.Hardware;
import frc.robot.controller.TestController;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.AutoSegment;
import frc.robot.autos.Points;
import frc.robot.autos.Trailblazer;
import frc.robot.autos.PointToPointAutos;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.util.ElasticLayoutUtil;
import frc.robot.util.scheduling.LifecycleSubsystemManager;
import frc.robot.vision.VisionSubsystem;
import frc.robot.AutoMovements.FieldPoints;
import frc.robot.fms.FmsSubsystem;
import frc.robot.currentPhase.phaseTimer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.AutoMovements.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.Orchestra;
import edu.wpi.first.math.geometry.Translation2d;



public class Robot extends TimedRobot {
  private static final boolean ASSUME_RED_ALLIANCE = false;
  private static final boolean ENABLE_DASHBOARD = true;
  private Command autonomousCommand = Commands.none();
  private final Hardware hardware = new Hardware();

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ImuSubsystem imu = new ImuSubsystem(swerve.drivetrainPigeon);

  private final VisionSubsystem vision = new VisionSubsystem(imu, hardware.leftLimelight, hardware.rightLimelight);
  private final LocalizationSubsystem localization = new LocalizationSubsystem(imu, vision, swerve);
  private final Trailblazer trailblazer = new Trailblazer(swerve, localization);
  private final HeadingLock headingLock = new HeadingLock(localization, swerve);
  private final DistanceCalc distanceCalc = new DistanceCalc(localization, headingLock);
  private final Flywheel flywheel = new Flywheel(hardware.flywheelA1, hardware.flywheelA2);
  private final Hood hood = new Hood(hardware.hoodMotor);
  private final LookupTable turretLookup = new LookupTable(distanceCalc, flywheel, hood);
  private final intaker intakeRoller = new intaker(hardware.intakeRollerMotor);
  private final IntakePosition intakePosition = new IntakePosition(hardware.intakePivotMotor);
  private final OutpostSetpoint outpost = new OutpostSetpoint(localization, swerve, intakePosition, intakeRoller);
  private final FlywheelStateMachine flywheelSM = new FlywheelStateMachine(flywheel);
  private final HoodStateMachine hoodSM = new HoodStateMachine(hood);
  private final Indexer indexer = new Indexer(hardware.indexerMotor);
  private final Hopper hopper = new Hopper(hardware.hopperMotor);
  private final EnterNeutralZone enterNeutralZone = new EnterNeutralZone(localization, trailblazer);
  private final TestController testController = new TestController(
    hardware.testController,
    hood,
    hoodSM,
    flywheel,
    flywheelSM,
    headingLock,
    intakePosition,
    indexer,
    hopper);



  private final phaseTimer phaseTimer = new phaseTimer();
  private final PointToPointAutos pointToPointAutos;
  private double tuningRpm = 3200.0;
  private final Orchestra orchestra = new Orchestra();
  private final edu.wpi.first.math.controller.PIDController trenchYController =
      new edu.wpi.first.math.controller.PIDController(3.0, 0.0, 0.0);
  private Translation2d savedRedTarget;
  private Translation2d savedBlueTarget;
  private Translation2d activePassTarget;
  private boolean rtShootMode = true;
  private phaseTimer.Phase lastPhase = null;
  private boolean warningRumbleSent = false;
  // Rumble pattern: array of {duration, pause, duration, pause, ...} in seconds
  // Negative values = rumble off (pause), positive = rumble on
  private double[] rumblePattern = null;
  private int rumblePatternIndex = 0;
  private double rumbleStepEndTime = 0;

  
  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(true);

    LifecycleSubsystemManager.ready();

    // Set up orchestra with all motors and load chrp file
    orchestra.addInstrument(hardware.flywheelA1);
    orchestra.addInstrument(hardware.flywheelA2);
    orchestra.addInstrument(hardware.hopperMotor);
    orchestra.addInstrument(hardware.hoodMotor);
    orchestra.addInstrument(hardware.indexerMotor);
    orchestra.addInstrument(hardware.intakePivotMotor);
    orchestra.addInstrument(hardware.intakeRollerMotor);
    orchestra.loadMusic("output.chrp");

    headingLock.setRedTargetPoint(FieldPoints.getHeadingLockRedPoint());
    headingLock.setBlueTargetPoint(FieldPoints.getHeadingLockBluePoint());

    headingLock.setLookupTable(turretLookup);

    registerNamedCommands();

    try {
      var ppConfig = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
        () -> localization.getPose(),                // Pose supplier
        (pose) -> localization.resetPose(pose),      // Pose reset
        () -> swerve.getRobotRelativeSpeeds(),       // Robot-relative speeds supplier
        (speeds, feedforwards) -> swerve.setRobotRelativeAutoSpeeds(speeds), // Drive robot-relative
        new PPHolonomicDriveController(
          new PIDConstants(5.0, 0.0, 0.0),           // Translation PID
          new PIDConstants(5.0, 0.0, 0.0)            // Rotation PID
        ),
        ppConfig,
        () -> FmsSubsystem.isRedAlliance(),          // Flip for red alliance
        swerve                                       // Drive subsystem requirement
      );
    } catch (Exception e) {
      DriverStation.reportError("Failed to configure PathPlanner: " + e.getMessage(), e.getStackTrace());
    }

    configureBindings();

    // Set up point-to-point auto chooser (shows on SmartDashboard as "Auto Chooser")
    pointToPointAutos = new PointToPointAutos(
        swerve, localization, flywheel, hood, flywheelSM, hoodSM,
        headingLock, turretLookup, indexer, hopper, intakeRoller, intakePosition);

    ElasticLayoutUtil.onBoot();
  }

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    ElasticLayoutUtil.onDisable();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // Use the point-to-point auto chooser from SmartDashboard
    autonomousCommand = pointToPointAutos.getSelected();
    CommandScheduler.getInstance().schedule(autonomousCommand);

    ElasticLayoutUtil.onEnable();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // Cancel all auto commands but keep the current pose
    CommandScheduler.getInstance().cancelAll();

    // Stop all mechanisms
    flywheel.stop();
    hood.stopMotor();
    indexer.stop();
    hopper.stop();
    intakeRoller.stop();
    intakePosition.retract();
    headingLock.disableLock();
    turretLookup.disable();
    flywheelSM.requestOff();
    hoodSM.requestOff();

    ElasticLayoutUtil.onEnable();

    phaseTimer.markTeleopStart();
    lastPhase = phaseTimer.getCurrentPhase();
    warningRumbleSent = false;
  }

  @Override
  public void teleopPeriodic() {
    phaseTimer.Phase currentPhase = phaseTimer.getCurrentPhase();
    double remaining = phaseTimer.getSecondsRemainingInCurrentPhase();

    // 5 seconds before shift ends: three quick rumble pulses
    if (remaining <= 5.0 && remaining > 4.5 && !warningRumbleSent && rumblePattern == null) {
      // Pattern: on 0.15s, off 0.1s, on 0.15s, off 0.1s, on 0.15s
      rumblePattern = new double[]{0.15, 0.1, 0.15, 0.1, 0.15};
      rumblePatternIndex = 0;
      rumbleStepEndTime = 0;
      warningRumbleSent = true;
    }

    // Shift change: one long rumble
    if (lastPhase != null && currentPhase != lastPhase) {
      rumblePattern = new double[]{0.8};
      rumblePatternIndex = 0;
      rumbleStepEndTime = 0;
      warningRumbleSent = false;
    }
    lastPhase = currentPhase;

    // Drive the rumble pattern
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    if (rumblePattern != null) {
      if (rumbleStepEndTime == 0) {
        // Start current step
        boolean isOn = (rumblePatternIndex % 2) == 0;
        hardware.driverController.getHID().setRumble(RumbleType.kBothRumble, isOn ? 1.0 : 0.0);
        rumbleStepEndTime = now + rumblePattern[rumblePatternIndex];
      } else if (now >= rumbleStepEndTime) {
        rumblePatternIndex++;
        if (rumblePatternIndex >= rumblePattern.length) {
          // Pattern done
          hardware.driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
          rumblePattern = null;
        } else {
          boolean isOn = (rumblePatternIndex % 2) == 0;
          hardware.driverController.getHID().setRumble(RumbleType.kBothRumble, isOn ? 1.0 : 0.0);
          rumbleStepEndTime = now + rumblePattern[rumblePatternIndex];
        }
      }
    }

    // Phase telemetry
    if (ENABLE_DASHBOARD) {
      SmartDashboard.putString("Phase/Current", currentPhase.name());
      SmartDashboard.putNumber("Phase/ElapsedSec", phaseTimer.getElapsedSec());
      SmartDashboard.putNumber("Phase/SecsInPhase", phaseTimer.getSecondsIntoCurrentPhase());
      SmartDashboard.putNumber("Phase/SecsRemaining", phaseTimer.getSecondsRemainingInCurrentPhase());
    }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  private void registerNamedCommands() {
    // Indexer states
    NamedCommands.registerCommand("IndexerOff", Commands.runOnce(() -> indexer.stop()));
    NamedCommands.registerCommand("IndexerIntake", Commands.runOnce(() -> indexer.intake()));
    NamedCommands.registerCommand("IndexerFeed", Commands.runOnce(() -> indexer.feed()));
    NamedCommands.registerCommand("IndexerReverse", Commands.runOnce(() -> indexer.reverse()));

    // Hopper states
    NamedCommands.registerCommand("HopperOff", Commands.runOnce(() -> hopper.stop()));
    NamedCommands.registerCommand("HopperIntake", Commands.runOnce(() -> hopper.intake()));
    NamedCommands.registerCommand("HopperFeed", Commands.runOnce(() -> hopper.feed()));
    NamedCommands.registerCommand("HopperReverse", Commands.runOnce(() -> hopper.reverse()));

    // Intaker (roller) states
    NamedCommands.registerCommand("IntakerOff", Commands.runOnce(() -> intakeRoller.stop()));
    NamedCommands.registerCommand("IntakerIntake", Commands.runOnce(() -> intakeRoller.intake()));
    NamedCommands.registerCommand("IntakerFeed", Commands.runOnce(() -> intakeRoller.feed()));
    NamedCommands.registerCommand("IntakerReverse", Commands.runOnce(() -> intakeRoller.reverse()));

    // Intake position states
    NamedCommands.registerCommand("IntakePositionDeploy", Commands.runOnce(() -> intakePosition.deploy()));
    NamedCommands.registerCommand("IntakePositionRetract", Commands.runOnce(() -> intakePosition.retract()));

    // Flywheel states
    NamedCommands.registerCommand("ShooterOff", Commands.runOnce(() -> flywheelSM.requestOff()));
    NamedCommands.registerCommand("ShooterSpin", Commands.runOnce(() -> flywheelSM.requestRpm(3200.0)));

    // Hood states
    NamedCommands.registerCommand("HoodOff", Commands.runOnce(() -> hoodSM.requestOff()));

    // Heading lock + lookup table: face target and spin up
    NamedCommands.registerCommand("FaceTarget", Commands.runOnce(() -> {
      headingLock.enableForAlliance();
      turretLookup.enable();
    }));
    NamedCommands.registerCommand("FaceTargetOff", Commands.runOnce(() -> {
      headingLock.disableLock();
      turretLookup.disable();
      flywheelSM.requestOff();
      hoodSM.requestOff();
    }));
  }

  private void configureBindings() {
    hardware.driverController.back().onTrue(
      Commands.runOnce(() -> {
        double heading = FmsSubsystem.isRedAlliance() ? 180.0 : 0.0;
        localization.resetGyro(Rotation2d.fromDegrees(heading));
      })
    );


    hardware.driverController.leftBumper().whileTrue(
      Commands.run(() -> {
        double robotY = localization.getPose().getY();
        double targetY = (Math.abs(robotY - 0.7) <= Math.abs(robotY - 7.45)) ? 0.7 : 7.45;
        double vy = trenchYController.calculate(robotY, targetY);
        swerve.overrideTeleopVY(vy);
        if (ENABLE_DASHBOARD) {
          SmartDashboard.putBoolean("Trench/Locked", true);
          SmartDashboard.putNumber("Trench/TargetY", targetY);
        }
      }).finallyDo(() -> {
        swerve.clearTeleopVYOverride();
        trenchYController.reset();
        if (ENABLE_DASHBOARD) SmartDashboard.putBoolean("Trench/Locked", false);
      })
    );

    

    hardware.driverController.leftTrigger(0.1).whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> {
          
        intakePosition.deploy();
        intakeRoller.intake();
        hopper.feed();

  
        },
        () -> {
              
        intakePosition.retract();
        intakeRoller.stop();
        hopper.stop();
    
        }
      )
    );


   hardware.driverController.x().whileTrue(
      outpost.travelToOutpost()
    );

  
    swerve.setDefaultCommand(
        swerve
            .run(
                () -> {
                  if (DriverStation.isTeleop()) {
                    swerve.driveTeleop(
                        hardware.driverController.getLeftX(),
                        hardware.driverController.getLeftY(),
                        hardware.driverController.getRightX());
                  }
                })
            .withName("DefaultSwerveCommand"));

    hardware.driverController.rightTrigger(0.1).whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> {
          double robotX = localization.getPose().getX();
          boolean shooting = robotX >= 11.0;
          rtShootMode = shooting;
          if (ENABLE_DASHBOARD) SmartDashboard.putBoolean("Driver/RT_ShootMode", shooting);

          if (shooting) {
            turretLookup.enable();
            headingLock.enableForAlliance();
            intakeRoller.intake();
            hopper.pulse();
            if (ENABLE_DASHBOARD) SmartDashboard.putBoolean("Driver/ShootingActive", true);
          } else {
            savedRedTarget = headingLock.getRedTargetPoint();
            savedBlueTarget = headingLock.getBlueTargetPoint();

            Translation2d robotPos = localization.getPose().getTranslation();
            double distRight = robotPos.getDistance(FieldPoints.PASS_TARGET_RIGHT);
            double distLeft = robotPos.getDistance(FieldPoints.PASS_TARGET_LEFT);
            activePassTarget = distRight < distLeft
                ? FieldPoints.PASS_TARGET_RIGHT : FieldPoints.PASS_TARGET_LEFT;

            headingLock.setRedTargetPoint(activePassTarget);
            headingLock.setBlueTargetPoint(activePassTarget);
            headingLock.enableForAlliance();
            hopper.pulse();
            if (ENABLE_DASHBOARD) SmartDashboard.putBoolean("Driver/PassingActive", true);
          }
        },
        () -> {
          boolean wasShootMode = rtShootMode;

          if (wasShootMode) {
            turretLookup.disable();
            headingLock.disableLock();
            flywheelSM.requestOff();
            hoodSM.requestOff();
            indexer.stop();
            intakeRoller.stop();
            hopper.stop();
            if (ENABLE_DASHBOARD) SmartDashboard.putBoolean("Driver/ShootingActive", false);
          } else {
            headingLock.setRedTargetPoint(savedRedTarget);
            headingLock.setBlueTargetPoint(savedBlueTarget);
            headingLock.disableLock();
            flywheel.stop();
            hood.setAngleDegrees(0);
            indexer.stop();
            intakeRoller.stop();
            hopper.stop();
            if (ENABLE_DASHBOARD) SmartDashboard.putBoolean("Driver/PassingActive", false);
          }
        }
      ).alongWith(
        edu.wpi.first.wpilibj2.command.Commands.run(() -> {
          boolean shootMode = rtShootMode;

          if (shootMode) {
            if (!turretLookup.hasCachedParameters()) return;
            double rpm = turretLookup.getCachedFlywheelRpm();
            double hoodRad = turretLookup.getCachedHoodAngleRad();
            flywheelSM.requestRpm(rpm);
            hoodSM.requestDegrees(Math.toDegrees(hoodRad));
            if (flywheel.isAtGoal() && headingLock.isSettled()) {
              indexer.feed();
              hopper.feed();
            }
          } else {
            double dist = localization.getPose().getTranslation().getDistance(activePassTarget);
            double rpm = LookupTable.getPassRpm(dist);
            flywheel.spinFlywheel(rpm);
            hood.setAngleDegrees(LookupTable.PASS_HOOD_ANGLE_DEG);
            if (ENABLE_DASHBOARD) {
              SmartDashboard.putNumber("Pass/Distance", dist);
              SmartDashboard.putNumber("Pass/RPM", rpm);
            }
            indexer.feed();
            hopper.feed();
          }
        })
      )
    );


  
  }
    
}