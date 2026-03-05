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
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.util.ElasticLayoutUtil;
import frc.robot.util.scheduling.LifecycleSubsystemManager;
import frc.robot.vision.VisionSubsystem;
import frc.robot.AutoMovements.FieldPoints;
import frc.robot.fms.FmsSubsystem;
import frc.robot.currentPhase.phaseTimer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private double tuningRpm = 3200.0;
  private final Orchestra orchestra = new Orchestra();
  private final edu.wpi.first.math.controller.PIDController trenchYController =
      new edu.wpi.first.math.controller.PIDController(3.0, 0.0, 0.0);
  private Translation2d savedRedTarget;
  private Translation2d savedBlueTarget;
  private Translation2d activePassTarget;

  
  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation());

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
    PathPlannerAuto auto = new PathPlannerAuto("New Auto");

    // Reset pose and heading to the auto's starting pose
    localization.resetPose(auto.getStartingPose());

    autonomousCommand = auto;
    CommandScheduler.getInstance().schedule(autonomousCommand);

    ElasticLayoutUtil.onEnable();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
           intakePosition.retract();


    ElasticLayoutUtil.onEnable();

    phaseTimer.markTeleopStart();
   // intakePosition.requestUp();
   // intakeRoller.stop();

    edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(localization.getZeroCommand());
  }

  @Override
  public void teleopPeriodic() {}

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
  }

  private void configureBindings() {
    hardware.driverController.back().onTrue(
      Commands.runOnce(() -> {
        double heading = FmsSubsystem.isRedAlliance() ? 180.0 : 0.0;
        localization.resetGyro(Rotation2d.fromDegrees(heading));
      })
    );

    // B button: play orchestra music while held, stop on release
    hardware.driverController.b().whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> {
          // Stop all subsystems that write to motors so Orchestra can control them
          flywheel.stop();
          hood.stopMotor();
          indexer.stop();
          hopper.stop();
          intakeRoller.stop();
          intakePosition.retract();
          orchestra.play();
        },
        () -> {
          orchestra.stop();
        }
      )
    );

    // Left stick button: lock Y to nearest trench (0.7 or 7.45) while held
    hardware.driverController.leftBumper().whileTrue(
      Commands.run(() -> {
        double robotY = localization.getPose().getY();
        double targetY = (Math.abs(robotY - 0.7) <= Math.abs(robotY - 7.45)) ? 0.7 : 7.45;
        double vy = trenchYController.calculate(robotY, targetY);
        swerve.overrideTeleopVY(vy);
        SmartDashboard.putBoolean("Trench/Locked", true);
        SmartDashboard.putNumber("Trench/TargetY", targetY);
      }).finallyDo(() -> {
        swerve.clearTeleopVYOverride();
        trenchYController.reset();
        SmartDashboard.putBoolean("Trench/Locked", false);
      })
    );

    
    hardware.driverController.x().whileTrue(
      Commands.run(() -> {
        flywheel.spinFlywheel(tuningRpm);
        if (flywheel.isAtGoal()) {
          indexer.setDutyPercent(1.0);
          hopper.setDutyPercent(0.7);
          intakeRoller.reverse();
        }
      }).finallyDo(() -> {
        flywheel.stop();
        indexer.stop();
        hopper.stop();
        intakeRoller.stop();
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


    

    hardware.driverController.povRight().whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> hood.runDutyCycle(0.3),
        () -> hood.stopMotor()
      )
    );

      hardware.driverController.povLeft().whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> hood.runDutyCycle(-0.3),
        () -> hood.stopMotor()
      )
    );


    hardware.driverController.povDown().whileTrue(
        Commands.runOnce(() -> {
         hood.setAngleDegrees(-60);
        }
      )
    );

        hardware.driverController.povUp().whileTrue(
        Commands.runOnce(() -> {
         hood.setAngleDegrees(0);
        }
      )
    );
/* 
    // D-pad Down → deploy elevator + turn intake rollers on
    hardware.driverController.povDown().onTrue(
      Commands.runOnce(() -> {
        intakePosition.deploy();
        intakeRoller.intake();
      })
    );

    // D-pad Up → retract elevator + turn intake rollers off
    hardware.driverController.povUp().onTrue(
      Commands.runOnce(() -> {
        intakePosition.retract();
        intakeRoller.stop();
      })
    );
    */

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
/* 
    hardware.driverController.rightTrigger().whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> {
          intakePosition.requestDown();
          intakeRoller.intake();
          SmartDashboard.putBoolean("Driver/IntakeIsDown", true);
        },
        () -> {
          intakePosition.requestUp();
          intakeRoller.stop();
          SmartDashboard.putBoolean("Driver/IntakeIsDown", false);
        }
      )
    );
*/
    hardware.driverController.rightTrigger(0.1).whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> {
          // On press: check X to decide shoot vs pass
          double robotX = localization.getPose().getX();
          boolean shooting = robotX >= 11.0;
          SmartDashboard.putBoolean("Driver/RT_ShootMode", shooting);

          if (shooting) {
            // --- SHOOT MODE (X >= 11) ---
            turretLookup.enable();
            headingLock.enableForAlliance();
            intakeRoller.intake();
            hopper.pulse();
            SmartDashboard.putBoolean("Driver/ShootingActive", true);
          } else {
            // --- PASS MODE (X < 11) ---
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
            SmartDashboard.putBoolean("Driver/PassingActive", true);
          }
        },
        () -> {
          // On release: clean up both modes
          boolean wasShootMode = SmartDashboard.getBoolean("Driver/RT_ShootMode", true);

          if (wasShootMode) {
            turretLookup.disable();
            headingLock.disableLock();
            flywheelSM.requestOff();
            hoodSM.requestOff();
            indexer.stop();
            intakeRoller.stop();
            hopper.stop();
            SmartDashboard.putBoolean("Driver/ShootingActive", false);
          } else {
            headingLock.setRedTargetPoint(savedRedTarget);
            headingLock.setBlueTargetPoint(savedBlueTarget);
            headingLock.disableLock();
            flywheel.stop();
            hood.setAngleDegrees(0);
            indexer.stop();
            intakeRoller.stop();
            hopper.stop();
            SmartDashboard.putBoolean("Driver/PassingActive", false);
          }
        }
      ).alongWith(
        edu.wpi.first.wpilibj2.command.Commands.run(() -> {
          boolean shootMode = SmartDashboard.getBoolean("Driver/RT_ShootMode", true);

          if (shootMode) {
            // --- SHOOT periodic ---
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
            // --- PASS periodic ---
            double dist = localization.getPose().getTranslation().getDistance(activePassTarget);
            double rpm = LookupTable.getPassRpm(dist);
            flywheel.spinFlywheel(rpm);
            hood.setAngleDegrees(LookupTable.PASS_HOOD_ANGLE_DEG);
            SmartDashboard.putNumber("Pass/Distance", dist);
            SmartDashboard.putNumber("Pass/RPM", rpm);
            if (flywheel.isAtGoal() && headingLock.isSettled()) {
              indexer.feed();
              hopper.feed();
            }
          }
        })
      )
    );


    hardware.driverController.rightBumper().whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> {
          // Save current shooting targets
          savedRedTarget = headingLock.getRedTargetPoint();
          savedBlueTarget = headingLock.getBlueTargetPoint();

          // Pick closest pass target
          Translation2d robotPos = localization.getPose().getTranslation();
          double distRight = robotPos.getDistance(FieldPoints.PASS_TARGET_RIGHT);
          double distLeft = robotPos.getDistance(FieldPoints.PASS_TARGET_LEFT);
          activePassTarget = distRight < distLeft
              ? FieldPoints.PASS_TARGET_RIGHT : FieldPoints.PASS_TARGET_LEFT;

          // Aim at pass target
          headingLock.setRedTargetPoint(activePassTarget);
          headingLock.setBlueTargetPoint(activePassTarget);
          headingLock.enableForAlliance();
          hopper.pulse();
          SmartDashboard.putBoolean("Driver/PassingActive", true);
        },
        () -> {
          // Restore original shooting targets
          headingLock.setRedTargetPoint(savedRedTarget);
          headingLock.setBlueTargetPoint(savedBlueTarget);
          headingLock.disableLock();
          flywheel.stop();
          hood.setAngleDegrees(0);
          indexer.stop();
          intakeRoller.stop();
          hopper.stop();
          SmartDashboard.putBoolean("Driver/PassingActive", false);
        }
      ).alongWith(
        edu.wpi.first.wpilibj2.command.Commands.run(() -> {
          // Distance-based pass interp
          double dist = localization.getPose().getTranslation().getDistance(activePassTarget);
          double rpm = LookupTable.getPassRpm(dist);
          flywheel.spinFlywheel(rpm);
          hood.setAngleDegrees(LookupTable.PASS_HOOD_ANGLE_DEG);
          SmartDashboard.putNumber("Pass/Distance", dist);
          SmartDashboard.putNumber("Pass/RPM", rpm);
          if (flywheel.isAtGoal() && headingLock.isSettled()) {
            indexer.feed();
            hopper.feed();
          }
        })
      )
    );
/* 
    hardware.driverController.y().whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> {
          turretLookup.enable();
          SmartDashboard.putBoolean("Driver/ShootingActive", true);
        },
        () -> {
          turretLookup.disable();
          flywheelSM.requestOff();
          hoodSM.requestOff();
          indexer.stop();
          hopper.stop();
          SmartDashboard.putBoolean("Driver/ShootingActive", false);
        }
      ).alongWith(
        edu.wpi.first.wpilibj2.command.Commands.run(() -> {
          if (!turretLookup.cachedParametersValid()) return;
          double rpm = turretLookup.getCachedFlywheelRpm();
          double hoodRad = turretLookup.getCachedHoodAngleRad();
          flywheelSM.requestRpm(rpm);
          hoodSM.requestDegrees(Math.toDegrees(hoodRad));
          indexer.feed();
          hopper.feed();
        })
      )
    );  */
  }
    
}