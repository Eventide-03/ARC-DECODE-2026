package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.FlywheelSubsystem.Flywheel;
import frc.robot.FlywheelSubsystem.FlywheelStateMachine;
import frc.robot.IndexerSubsystem.Hopper;
import frc.robot.IndexerSubsystem.Indexer;
import frc.robot.Intake.IntakePosition;
import frc.robot.Intake.intaker;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.limelight.LimelightHelpers;

public class RobotContainer {
    private static final double TURN_RATE_RPS = 1.275; // 1.7x of prior 0.75 rps baseline
    private static final double SHOOT_NO_AUTOALIGN_RPM = 4050.0;
    private static final double SHOOT_FEED_DELAY_SEC = 0.3;
    private static final double SHOOT_INTAKE_DUTY = 0.4;
    private static final String SHOOTER_LIMELIGHT = "limelight-shooter";

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(TURN_RATE_RPS).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentricFacingAngle autoAlignDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withHeadingPID(7.0, 0.0, 0.15);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final Hardware hardware = new Hardware();
    private final intaker intakeRoller = new intaker(hardware.intakeRollerMotor);
    private final IntakePosition intakePivot = new IntakePosition(hardware.intakePivotMotor);
    private final Hopper hopper = new Hopper(hardware.hopperMotor);
    private final Indexer indexer = new Indexer(hardware.indexerMotor);
    private final Flywheel flywheel = new Flywheel(hardware.flywheelA1, hardware.flywheelA2);
    private final FlywheelStateMachine flywheelSM = new FlywheelStateMachine(flywheel);
    private final Timer shootFeedDelayTimer = new Timer();
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private Rotation2d m_lastAutoAlignHeading = Rotation2d.kZero;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureAutoChooser();
        configureBindings();
    }

    private void configureAutoChooser() {
        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        autoChooser.addOption("Drive Forward (5s)", createDriveForwardAuto());

        Shuffleboard.getTab("Autonomous")
                .add("Auto Chooser", autoChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(0, 0)
                .withSize(4, 2);
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Left trigger: intake.
        joystick.leftTrigger().whileTrue(intakeCommand());

        // Y: shoot without autoalign at fixed 4050 RPM.
        joystick.y().whileTrue(shootAtRpmCommand(SHOOT_NO_AUTOALIGN_RPM));

        // Right trigger: shoot with autoalign, plus delayed feed.
        joystick.rightTrigger().whileTrue(
            Commands.parallel(
                drivetrain.applyRequest(() ->
                    autoAlignDrive
                        .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withTargetDirection(getAutoAlignHeading())
                ),
                shootAtRpmCommand(SHOOT_NO_AUTOALIGN_RPM)
            )
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Back + Start (the two squares): hard reset heading to 0 so AdvantageScope theta goes to zero.
        joystick.back().and(joystick.start()).onTrue(
            drivetrain.runOnce(() -> {
                drivetrain.resetRotation(Rotation2d.kZero);
                drivetrain.seedFieldCentric(Rotation2d.kZero);
                m_lastAutoAlignHeading = Rotation2d.kZero;
            })
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private Rotation2d getAutoAlignHeading() {
        Rotation2d currentHeading = drivetrain.getState().Pose.getRotation();
        if (LimelightHelpers.getTV(SHOOTER_LIMELIGHT)) {
            double txDegrees = LimelightHelpers.getTX(SHOOTER_LIMELIGHT);
            m_lastAutoAlignHeading = currentHeading.plus(Rotation2d.fromDegrees(-txDegrees));
        } else {
            m_lastAutoAlignHeading = currentHeading;
        }
        return m_lastAutoAlignHeading;
    }

    private Command intakeCommand() {
        return Commands.startEnd(
            () -> {
                intakePivot.deploy();
                intakeRoller.intake();
                hopper.intake();
                indexer.intake();
            },
            () -> {
                intakeRoller.stop();
                hopper.stop();
                indexer.stop();
                intakePivot.retract();
            },
            intakeRoller, hopper, indexer, intakePivot
        );
    }

    private Command shootAtRpmCommand(double rpm) {
        return Commands.run(
            () -> {
                flywheelSM.requestRpm(rpm);
                if (shootFeedDelayTimer.hasElapsed(SHOOT_FEED_DELAY_SEC)) {
                    intakeRoller.setDutyPercent(SHOOT_INTAKE_DUTY);
                    hopper.feed();
                    indexer.feed();
                }
            },
            intakeRoller, hopper, indexer
        ).beforeStarting(shootFeedDelayTimer::restart)
         .finallyDo(interrupted -> {
             shootFeedDelayTimer.stop();
             shootFeedDelayTimer.reset();
             flywheelSM.requestOff();
             intakeRoller.stop();
             hopper.stop();
             indexer.stop();
         });
    }

    /** Ensures all mechanisms are stopped so re-enable never resumes prior motor output. */
    public void stopAndResetForDisable() {
        drivetrain.setControl(new SwerveRequest.Idle());
        flywheelSM.requestOff();
        intakeRoller.stop();
        hopper.stop();
        indexer.stop();
        intakePivot.retract();
        shootFeedDelayTimer.stop();
        shootFeedDelayTimer.reset();
    }

    public Command getAutonomousCommand() {
        Command selected = autoChooser.getSelected();
        return selected != null ? selected : createDriveForwardAuto();
    }

    private Command createDriveForwardAuto() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}

