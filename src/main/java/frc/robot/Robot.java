package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
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
import frc.robot.currentPhase.phaseTimer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
  private final OutpostSetpoint outpost = new OutpostSetpoint(localization, trailblazer);
  private final DistanceCalc distanceCalc = new DistanceCalc(localization, headingLock);
  private final Flywheel flywheel = new Flywheel(hardware.flywheelA1, hardware.flywheelA2);
  private final Hood hood = new Hood(hardware.hoodMotor);
  private final LookupTable turretLookup = new LookupTable(distanceCalc, flywheel, hood);
  private final intaker intakeRoller = new intaker(hardware.intakeRollerMotor);
  private final IntakePosition intakePosition = new IntakePosition(hardware.intakePivotMotor);
  private final FlywheelStateMachine flywheelSM = new FlywheelStateMachine(flywheel);
  private final HoodStateMachine hoodSM = new HoodStateMachine(hood);
  private final Indexer indexer = new Indexer(hardware.indexerMotor);
  private final Hopper hopper = new Hopper(hardware.hopperMotor);
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
  
  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation());

    LifecycleSubsystemManager.ready();

    headingLock.setRedTargetPoint(FieldPoints.getHeadingLockRedPoint());
    headingLock.setBlueTargetPoint(FieldPoints.getHeadingLockBluePoint());

    headingLock.setLookupTable(turretLookup);

    configureBindings();

    ElasticLayoutUtil.onBoot();
  }

  @Override
  public void robotInit() {}

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
    var isRed = ASSUME_RED_ALLIANCE;
    var startPose = isRed ? Points.START_R1_AND_B1_FORWARD.redPose : Points.START_R1_AND_B1_FORWARD.bluePose;
    var endPose = new edu.wpi.first.math.geometry.Pose2d(15.0, startPose.getY(), edu.wpi.first.math.geometry.Rotation2d.kZero);

    localization.resetPose(startPose);

    var constraints = new AutoConstraintOptions();
    var segment = new AutoSegment(constraints, new AutoPoint(startPose), new AutoPoint(endPose));

    autonomousCommand = trailblazer
      .followSegment(segment, true)
      .withName((isRed ? "Red" : "Blue") + "_A_to_B_Auto");

    edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(autonomousCommand);

    ElasticLayoutUtil.onEnable();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    autonomousCommand.cancel();

    ElasticLayoutUtil.onEnable();

    phaseTimer.markTeleopStart();
    intakePosition.requestUp();
    intakeRoller.stop();

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

  private void configureBindings() {
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

    hardware.driverController.y().whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> {
          turretLookup.enable();
          headingLock.enableForAlliance();
          SmartDashboard.putBoolean("Driver/ShootingActive", true);
        },
        () -> {
          turretLookup.disable();
          headingLock.disableLock();
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
    );

    hardware.driverController.leftTrigger().whileTrue(
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
    );
  }
}