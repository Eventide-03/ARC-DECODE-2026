package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
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

import frc.robot.AutoMovements.FieldPoints;
import frc.robot.FlywheelSubsystem.Flywheel;
import frc.robot.FlywheelSubsystem.FlywheelStateMachine;
import frc.robot.IndexerSubsystem.Hopper;
import frc.robot.IndexerSubsystem.Indexer;
import frc.robot.Intake.IntakePosition;
import frc.robot.Intake.intaker;
import frc.robot.fms.FmsSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.limelight.LimelightHelpers;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75 * 1.5 * 1.25).in(RadiansPerSecond); // 1.25x faster turning than current setting
    private static final double DEFAULT_SHOOT_RPM = 4050.0;
    private static final double SHOOT_RPM_STEP = 100.0;
    private static final double MIN_SHOOT_RPM = 1500.0;
    private static final double MAX_SHOOT_RPM = 6500.0;
    private static final double AUTO_ALIGN_FALLBACK_RPM = DEFAULT_SHOOT_RPM;
    private static final double LIMELIGHT_DISTANCE_WEIGHT = 0.70;
    private static final double POSE_DISTANCE_WEIGHT = 0.30;

    // Distance (m) to RPM points for auto-align shooting.
    private static final double[] AUTO_RPM_DISTANCES_M = {2.8, 3.2, 3.6, 4.0, 4.9, 5.1};
    private static final double[] AUTO_RPM_VALUES = {3100.0, 3200.0, 3390.0, 3500.0, 3900.0, 4100.0};

    // Vision-assisted auto-align tuning. Shooter camera is weighted higher than side cameras.
    private static final String SHOOTER_LIMELIGHT = "limelight-shooter";
    private static final String LEFT_LIMELIGHT = "limelight-left";
    private static final String RIGHT_LIMELIGHT = "limelight-right";
    private static final double SHOOTER_TX_WEIGHT = 0.85;
    private static final double SIDE_TX_WEIGHT = 0.15;
    private static final double MAX_VISION_CORRECTION_DEG = 8.0;
    private static final double VISION_CORRECTION_SLEW_DEG_PER_SEC = 240.0;
    private static final double VISION_XY_STD_DEV_METERS = 0.7;
    private static final double VISION_THETA_STD_DEV_RADIANS = 999.0;
    private static final double MAX_VISION_JUMP_METERS = 2.5;
    private static final double MAX_VISION_FIRST_LOCK_JUMP_METERS = 12.0;

    private static final double SHOOT_HOPPER_DUTY = 0.50;
    private static final double SHOOT_INDEXER_DUTY = 0.75;
    private static final double SHOOT_INTAKE_ROLLER_DUTY = 0.40;
    private static final double SHOOT_FEED_DELAY_SEC = 0.30;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.FieldCentric routeDrive = new SwerveRequest.FieldCentric()
            .withDeadband(0)
            .withRotationalDeadband(0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
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
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private final Timer indexPulseTimer = new Timer();
    private final Timer shootFeedDelayTimer = new Timer();
    private double noAutoAlignTargetRpm = DEFAULT_SHOOT_RPM;
    private double autoAlignLookupRpmOffset = 0.0;
    private double lastVisionCorrectionDeg = 0.0;
    private double lastVisionTimestampSec = Timer.getFPGATimestamp();
    private boolean hasAcceptedVisionSinceEnable = false;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureAutoChooser();
        configureBindings();
        SmartDashboard.putNumber("Shooter/NoAutoAlignTargetRPM", noAutoAlignTargetRpm);
        SmartDashboard.putNumber("Shooter/AutoAlignLookupRpmOffset", autoAlignLookupRpmOffset);
        SmartDashboard.putNumber("Shooter/AutoAlignTargetRPM", AUTO_ALIGN_FALLBACK_RPM);
        SmartDashboard.putNumber("Shooter/AutoAlignLookupBaseRPM", AUTO_ALIGN_FALLBACK_RPM);
        SmartDashboard.putNumber("Shooter/AutoAlignTargetDistanceM", 0.0);
        SmartDashboard.putNumber("Shooter/AutoAlignPoseDistanceM", 0.0);
        SmartDashboard.putNumber("Shooter/AutoAlignLimelightDistanceM", -1.0);
        SmartDashboard.putString("Shooter/AutoAlignDistanceSource", "pose-only");
        SmartDashboard.putNumber("Shooter/AutoAlignVisionCorrectionDeg", 0.0);
        SmartDashboard.putString("Shooter/AutoAlignVisionSource", "none");
        SmartDashboard.putString("Localization/VisionLastSource", "none");
        SmartDashboard.putNumber("Localization/VisionLastTagCount", 0);
        SmartDashboard.putString("Localization/VisionLastStatus", "idle");
    }

    /** Periodic hook for container-level updates that are not command-owned. */
    public void robotPeriodic() {
        applyBestVisionMeasurement();
    }

    private void configureAutoChooser() {
        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        autoChooser.addOption("Drive Forward (5s)", createDriveForwardAuto());
        autoChooser.addOption("Red Left Path", createRedLeftPathAuto());
        autoChooser.addOption("Red Right Path", createRedRightPathAuto());

        // Publish chooser in both places so it shows up reliably in Shuffleboard and SmartDashboard.
        Shuffleboard.getTab("Autonomous")
                .add("Auto Chooser", autoChooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withPosition(0, 0)
                .withSize(4, 2);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putBoolean("AutoChooser/Published", true);
    }

    /** Force all mechanism outputs to zero and reset transient control state. */
    public void stopAndResetForDisable() {
        // Swerve stop
        drivetrain.setControl(new SwerveRequest.Idle());

        // Mechanism stop
        flywheelSM.requestOff();
        hopper.stop();
        indexer.stop();
        intakeRoller.stop();
        intakePivot.retract();

        // Reset helper state so nothing carries into next enable
        indexPulseTimer.stop();
        lastVisionCorrectionDeg = 0.0;
        lastVisionTimestampSec = Timer.getFPGATimestamp();
        hasAcceptedVisionSinceEnable = false;
        SmartDashboard.putNumber("Shooter/AutoAlignVisionCorrectionDeg", 0.0);
        SmartDashboard.putString("Shooter/AutoAlignVisionSource", "none");
        SmartDashboard.putString("Localization/VisionLastStatus", "idle");
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

        // X: index routine with hopper pulse reverse every 2.5s for 0.5s.
        joystick.x().whileTrue(indexCommand());

        // Left trigger: intake.
        joystick.leftTrigger().whileTrue(intakeCommand());

        // Right trigger: shoot with auto-align.
        joystick.rightTrigger().whileTrue(shootAutoAlignCommand());

        // Y: shoot without auto-align, reset no-auto-align target to 3910 each press.
        joystick.y().onTrue(Commands.runOnce(() -> setNoAutoAlignTargetRpm(DEFAULT_SHOOT_RPM)));
        joystick.y().whileTrue(shootNoAutoAlignCommand());

        // D-pad up/down: change no-auto-align target RPM while robot is enabled.
        joystick.povUp().onTrue(Commands.runOnce(() -> {
            if (DriverStation.isEnabled()) {
                setNoAutoAlignTargetRpm(noAutoAlignTargetRpm + SHOOT_RPM_STEP);
            }
        }));
        joystick.povDown().onTrue(Commands.runOnce(() -> {
            if (DriverStation.isEnabled()) {
                setNoAutoAlignTargetRpm(noAutoAlignTargetRpm - SHOOT_RPM_STEP);
            }
        }));

        // D-pad left/right: trim auto-align lookup RPM table output by +/-100 while enabled.
        joystick.povLeft().onTrue(Commands.runOnce(() -> {
            if (DriverStation.isEnabled()) {
                setAutoAlignLookupRpmOffset(autoAlignLookupRpmOffset - SHOOT_RPM_STEP);
            }
        }));
        joystick.povRight().onTrue(Commands.runOnce(() -> {
            if (DriverStation.isEnabled()) {
                setAutoAlignLookupRpmOffset(autoAlignLookupRpmOffset + SHOOT_RPM_STEP);
            }
        }));

        // B: route to outpost.
        joystick.b().onTrue(routeToOutpostCommand());

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Back + Start (the two square buttons): hard reset heading to 0 deg.
        joystick.back().and(joystick.start()).onTrue(
            drivetrain.runOnce(() -> {
                drivetrain.resetRotation(Rotation2d.kZero);
                drivetrain.seedFieldCentric(Rotation2d.kZero);
            })
        );

        // SysId routines.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void setNoAutoAlignTargetRpm(double rpm) {
        noAutoAlignTargetRpm = MathUtil.clamp(rpm, MIN_SHOOT_RPM, MAX_SHOOT_RPM);
        SmartDashboard.putNumber("Shooter/NoAutoAlignTargetRPM", noAutoAlignTargetRpm);
    }

    private void setAutoAlignLookupRpmOffset(double rpmOffset) {
        autoAlignLookupRpmOffset = rpmOffset;
        SmartDashboard.putNumber("Shooter/AutoAlignLookupRpmOffset", autoAlignLookupRpmOffset);
    }

    private Command intakeCommand() {
        return Commands.startEnd(
            () -> {
                intakePivot.deploy();
                intakeRoller.setDutyPercent(1.0);
            },
            () -> {
                intakeRoller.stop();
                intakePivot.retract();
            },
            intakePivot, intakeRoller
        );
    }

    private Command indexCommand() {
        return Commands.run(
            () -> {
                double cycleSeconds = indexPulseTimer.get() % 3.0;
                double hopperDuty = cycleSeconds >= 2.5 ? -0.5 : 0.5;

                intakeRoller.setDutyPercent(1.0);
                hopper.setDutyPercent(hopperDuty);
                indexer.setDutyPercent(0.75);
            },
            intakeRoller, hopper, indexer
        ).beforeStarting(indexPulseTimer::restart)
         .finallyDo(interrupted -> {
             indexPulseTimer.stop();
             intakeRoller.stop();
             hopper.stop();
             indexer.stop();
         });
    }

    private Rotation2d getAutoAlignTargetDirection() {
        Translation2d target = FmsSubsystem.isRedAlliance()
                ? FieldPoints.getHeadingLockRedPoint()
                : FieldPoints.getHeadingLockBluePoint();

        Translation2d robot = drivetrain.getState().Pose.getTranslation();
        Rotation2d baseTargetDirection = target.minus(robot).getAngle();

        double shooterTx = readTxIfValid(SHOOTER_LIMELIGHT);
        double sideLeftTx = readTxIfValid(LEFT_LIMELIGHT);
        double sideRightTx = readTxIfValid(RIGHT_LIMELIGHT);

        double sideTx = 0.0;
        boolean hasLeft = !Double.isNaN(sideLeftTx);
        boolean hasRight = !Double.isNaN(sideRightTx);
        boolean hasShooter = !Double.isNaN(shooterTx);
        if (hasLeft && hasRight) {
            sideTx = (sideLeftTx + sideRightTx) * 0.5;
        } else if (hasLeft) {
            sideTx = sideLeftTx;
        } else if (hasRight) {
            sideTx = sideRightTx;
        }

        // Negative sign maps positive TX (target right in image) to a right-turn correction.
        double shooterCorrectionDeg = hasShooter ? -shooterTx : 0.0;
        double sideCorrectionDeg = (hasLeft || hasRight) ? -sideTx : 0.0;

        // Prioritize shooter camera to avoid side-camera induced over-turn before release.
        double desiredCorrectionDeg;
        String correctionSource;
        if (hasShooter) {
            desiredCorrectionDeg = shooterCorrectionDeg;
            correctionSource = "shooter";
        } else if (hasLeft || hasRight) {
            desiredCorrectionDeg =
                    (SHOOTER_TX_WEIGHT * shooterCorrectionDeg) + (SIDE_TX_WEIGHT * sideCorrectionDeg);
            correctionSource = hasLeft && hasRight ? "left+right" : (hasLeft ? "left" : "right");
        } else {
            desiredCorrectionDeg = 0.0;
            correctionSource = "field-fallback";
        }

        desiredCorrectionDeg = MathUtil.clamp(
                desiredCorrectionDeg,
                -MAX_VISION_CORRECTION_DEG,
                MAX_VISION_CORRECTION_DEG
        );

        double nowSec = Timer.getFPGATimestamp();
        double dtSec = Math.max(0.0, nowSec - lastVisionTimestampSec);
        lastVisionTimestampSec = nowSec;

        double maxStepDeg = VISION_CORRECTION_SLEW_DEG_PER_SEC * dtSec;
        double deltaDeg = MathUtil.clamp(
                desiredCorrectionDeg - lastVisionCorrectionDeg,
                -maxStepDeg,
                maxStepDeg
        );
        lastVisionCorrectionDeg += deltaDeg;

        SmartDashboard.putNumber("Shooter/AutoAlignVisionCorrectionDeg", lastVisionCorrectionDeg);
        SmartDashboard.putString("Shooter/AutoAlignVisionSource", correctionSource);

        return baseTargetDirection.plus(Rotation2d.fromDegrees(lastVisionCorrectionDeg));
    }

    private double readTxIfValid(String limelightName) {
        return LimelightHelpers.getTV(limelightName)
                ? LimelightHelpers.getTX(limelightName)
                : Double.NaN;
    }

    private LimelightHelpers.PoseEstimate getBestEstimateForCamera(String limelightName) {
        LimelightHelpers.PoseEstimate standard = FmsSubsystem.isRedAlliance()
                ? LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName)
                : LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        LimelightHelpers.PoseEstimate megaTag2 = FmsSubsystem.isRedAlliance()
                ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName)
                : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        boolean standardValid = standard != null && standard.pose != null && standard.tagCount > 0;
        boolean megaTag2Valid = megaTag2 != null && megaTag2.pose != null && megaTag2.tagCount > 0;

        if (standardValid && megaTag2Valid) {
            // Prefer more tags; tie-break by closer average tag distance.
            if (megaTag2.tagCount > standard.tagCount) {
                return megaTag2;
            }
            if (megaTag2.tagCount < standard.tagCount) {
                return standard;
            }
            return megaTag2.avgTagDist <= standard.avgTagDist ? megaTag2 : standard;
        }
        if (megaTag2Valid) {
            return megaTag2;
        }
        return standardValid ? standard : null;
    }

    private void applyBestVisionMeasurement() {
        String bestSource = null;
        LimelightHelpers.PoseEstimate bestEstimate = null;

        String[] cameraOrder = {SHOOTER_LIMELIGHT, LEFT_LIMELIGHT, RIGHT_LIMELIGHT};
        for (String limelightName : cameraOrder) {
            LimelightHelpers.PoseEstimate candidate = getBestEstimateForCamera(limelightName);
            if (candidate == null) {
                continue;
            }

            if (bestEstimate == null
                    || candidate.tagCount > bestEstimate.tagCount
                    || (candidate.tagCount == bestEstimate.tagCount && candidate.avgTagDist < bestEstimate.avgTagDist)) {
                bestEstimate = candidate;
                bestSource = limelightName;
            }
        }

        if (bestEstimate == null || bestSource == null) {
            SmartDashboard.putString("Localization/VisionLastStatus", "no-tags");
            return;
        }

        Pose2d currentPose = drivetrain.getState().Pose;
        Pose2d xyOnlyVisionPose = new Pose2d(bestEstimate.pose.getTranslation(), currentPose.getRotation());

        // Reject large one-frame jumps from bad solves to avoid field spin/jitter in odometry.
        double translationJump = currentPose.getTranslation().getDistance(xyOnlyVisionPose.getTranslation());
        double allowedJumpMeters = hasAcceptedVisionSinceEnable
                ? MAX_VISION_JUMP_METERS
                : MAX_VISION_FIRST_LOCK_JUMP_METERS;
        if (translationJump > allowedJumpMeters) {
            SmartDashboard.putString("Localization/VisionLastStatus", "rejected-jump");
            return;
        }

        drivetrain.addVisionMeasurement(
                xyOnlyVisionPose,
                bestEstimate.timestampSeconds,
                VecBuilder.fill(
                        VISION_XY_STD_DEV_METERS,
                        VISION_XY_STD_DEV_METERS,
                        VISION_THETA_STD_DEV_RADIANS
                )
        );
        hasAcceptedVisionSinceEnable = true;

        SmartDashboard.putString("Localization/VisionLastSource", bestSource);
        SmartDashboard.putNumber("Localization/VisionLastTagCount", bestEstimate.tagCount);
        SmartDashboard.putString("Localization/VisionLastStatus", "accepted");
    }

    private double estimateDistanceFromPoseMeters() {
        Translation2d target = FmsSubsystem.isRedAlliance()
                ? FieldPoints.getHeadingLockRedPoint()
                : FieldPoints.getHeadingLockBluePoint();
        return drivetrain.getState().Pose.getTranslation().getDistance(target);
    }

    private double readShooterLimelightDistanceMeters() {
        if (!LimelightHelpers.getTV(SHOOTER_LIMELIGHT)) {
            return Double.NaN;
        }

        double[] targetPoseRobotSpace = LimelightHelpers.getTargetPose_RobotSpace(SHOOTER_LIMELIGHT);
        if (targetPoseRobotSpace.length < 2) {
            return Double.NaN;
        }

        // Use planar robot-space distance to the tracked target.
        double x = targetPoseRobotSpace[0];
        double y = targetPoseRobotSpace[1];
        return Math.hypot(x, y);
    }

    private double interpolateAutoAlignRpm(double distanceMeters) {
        if (AUTO_RPM_DISTANCES_M.length == 0 || AUTO_RPM_VALUES.length == 0
                || AUTO_RPM_DISTANCES_M.length != AUTO_RPM_VALUES.length) {
            return AUTO_ALIGN_FALLBACK_RPM;
        }

        if (distanceMeters <= AUTO_RPM_DISTANCES_M[0]) {
            return AUTO_RPM_VALUES[0];
        }

        int last = AUTO_RPM_DISTANCES_M.length - 1;
        if (distanceMeters >= AUTO_RPM_DISTANCES_M[last]) {
            return AUTO_RPM_VALUES[last];
        }

        for (int i = 1; i < AUTO_RPM_DISTANCES_M.length; i++) {
            double d0 = AUTO_RPM_DISTANCES_M[i - 1];
            double d1 = AUTO_RPM_DISTANCES_M[i];
            if (distanceMeters <= d1) {
                double t = (distanceMeters - d0) / (d1 - d0);
                double rpm = AUTO_RPM_VALUES[i - 1] + (AUTO_RPM_VALUES[i] - AUTO_RPM_VALUES[i - 1]) * t;
                return MathUtil.clamp(rpm, MIN_SHOOT_RPM, MAX_SHOOT_RPM);
            }
        }

        return AUTO_ALIGN_FALLBACK_RPM;
    }

    private double getAutoAlignTargetRpm() {
        double poseDistance = estimateDistanceFromPoseMeters();
        double limelightDistance = readShooterLimelightDistanceMeters();

        double fusedDistance;
        String source;
        if (!Double.isNaN(limelightDistance) && limelightDistance > 0.0) {
            fusedDistance = (LIMELIGHT_DISTANCE_WEIGHT * limelightDistance) + (POSE_DISTANCE_WEIGHT * poseDistance);
            source = "fused";
        } else {
            fusedDistance = poseDistance;
            source = "pose-only";
        }

        double baseRpm = interpolateAutoAlignRpm(fusedDistance);
        double rpm = MathUtil.clamp(baseRpm + autoAlignLookupRpmOffset, MIN_SHOOT_RPM, MAX_SHOOT_RPM);
        SmartDashboard.putNumber("Shooter/AutoAlignPoseDistanceM", poseDistance);
        SmartDashboard.putNumber(
                "Shooter/AutoAlignLimelightDistanceM",
                Double.isNaN(limelightDistance) ? -1.0 : limelightDistance
        );
        SmartDashboard.putNumber("Shooter/AutoAlignTargetDistanceM", fusedDistance);
        SmartDashboard.putNumber("Shooter/AutoAlignLookupBaseRPM", baseRpm);
        SmartDashboard.putNumber("Shooter/AutoAlignTargetRPM", rpm);
        SmartDashboard.putString("Shooter/AutoAlignDistanceSource", source);
        return rpm;
    }

    private Command shootAutoAlignCommand() {
        return Commands.run(
            () -> {
                flywheelSM.requestRpm(getAutoAlignTargetRpm());
                intakeRoller.setDutyPercent(SHOOT_INTAKE_ROLLER_DUTY);

                if (shootFeedDelayTimer.hasElapsed(SHOOT_FEED_DELAY_SEC)) {
                    hopper.setDutyPercent(SHOOT_HOPPER_DUTY);
                    indexer.setDutyPercent(SHOOT_INDEXER_DUTY);
                } else {
                    hopper.stop();
                    indexer.stop();
                }

                drivetrain.setControl(
                    autoAlignDrive
                        .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withTargetDirection(getAutoAlignTargetDirection())
                );
            },
            drivetrain, flywheelSM, hopper, indexer, intakeRoller
        ).beforeStarting(() -> {
            lastVisionTimestampSec = Timer.getFPGATimestamp();
            lastVisionCorrectionDeg = 0.0;
            shootFeedDelayTimer.restart();
        }).finallyDo(interrupted -> {
            flywheelSM.requestOff();
            intakeRoller.stop();
            hopper.stop();
            indexer.stop();
            shootFeedDelayTimer.stop();
            lastVisionCorrectionDeg = 0.0;
        });
    }

    private Command shootNoAutoAlignCommand() {
        return Commands.run(
            () -> {
                flywheelSM.requestRpm(noAutoAlignTargetRpm);
                intakeRoller.setDutyPercent(SHOOT_INTAKE_ROLLER_DUTY);

                if (shootFeedDelayTimer.hasElapsed(SHOOT_FEED_DELAY_SEC)) {
                    hopper.setDutyPercent(SHOOT_HOPPER_DUTY);
                    indexer.setDutyPercent(SHOOT_INDEXER_DUTY);
                } else {
                    hopper.stop();
                    indexer.stop();
                }
            },
            flywheelSM, hopper, indexer, intakeRoller
        ).beforeStarting(shootFeedDelayTimer::restart)
         .finallyDo(interrupted -> {
             flywheelSM.requestOff();
             intakeRoller.stop();
             hopper.stop();
             indexer.stop();
             shootFeedDelayTimer.stop();
         });
    }

    private boolean isAtOutpost() {
        Pose2d target = FmsSubsystem.isRedAlliance() ? FieldPoints.getOutpostRed() : FieldPoints.getOutpostBlue();
        Pose2d current = drivetrain.getState().Pose;

        double xyError = current.getTranslation().getDistance(target.getTranslation());
        double thetaError = Math.abs(target.getRotation().minus(current.getRotation()).getDegrees());
        return xyError < 0.15 && thetaError < 7.0;
    }

    private Command routeToOutpostCommand() {
        final var idle = new SwerveRequest.Idle();

        return Commands.run(
            () -> {
                Pose2d target = FmsSubsystem.isRedAlliance() ? FieldPoints.getOutpostRed() : FieldPoints.getOutpostBlue();
                Pose2d current = drivetrain.getState().Pose;

                double xError = target.getX() - current.getX();
                double yError = target.getY() - current.getY();
                double thetaErrorRad = target.getRotation().minus(current.getRotation()).getRadians();

                double vx = MathUtil.clamp(xError * 1.4, -1.75, 1.75);
                double vy = MathUtil.clamp(yError * 1.4, -1.75, 1.75);
                double omega = MathUtil.clamp(thetaErrorRad * 3.0, -2.5, 2.5);

                drivetrain.setControl(
                    routeDrive.withVelocityX(vx)
                             .withVelocityY(vy)
                             .withRotationalRate(omega)
                );
            },
            drivetrain
        ).until(this::isAtOutpost)
         .finallyDo(interrupted -> drivetrain.setControl(idle));
    }

    public Command getAutonomousCommand() {
        Command selected = autoChooser.getSelected();
        return selected != null ? selected : Commands.none();
    }

    private Command createDriveForwardAuto() {
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

    private Command driveToPoseStep(Pose2d targetPose, double timeoutSeconds) {
        final var idle = new SwerveRequest.Idle();

        return Commands.run(
            () -> {
                Pose2d current = drivetrain.getState().Pose;

                double xError = targetPose.getX() - current.getX();
                double yError = targetPose.getY() - current.getY();
                double thetaErrorRad = targetPose.getRotation().minus(current.getRotation()).getRadians();

                double vx = MathUtil.clamp(xError * 1.4, -2.0, 2.0);
                double vy = MathUtil.clamp(yError * 1.4, -2.0, 2.0);
                double omega = MathUtil.clamp(thetaErrorRad * 3.2, -3.0, 3.0);

                drivetrain.setControl(
                    routeDrive.withVelocityX(vx)
                             .withVelocityY(vy)
                             .withRotationalRate(omega)
                );
            },
            drivetrain
        ).until(() -> {
            Pose2d current = drivetrain.getState().Pose;
            double xyError = current.getTranslation().getDistance(targetPose.getTranslation());
            double thetaError = Math.abs(targetPose.getRotation().minus(current.getRotation()).getDegrees());
            return xyError < 0.20 && thetaError < 8.0;
        }).withTimeout(timeoutSeconds)
         .finallyDo(interrupted -> drivetrain.setControl(idle));
    }

    private Command createRedLeftPathAuto() {
        Pose2d start = new Pose2d(12.84, 0.7, Rotation2d.fromDegrees(180.0));
        return Commands.sequence(
            Commands.runOnce(() -> drivetrain.resetPose(start)),
            driveToPoseStep(new Pose2d(8.8, 0.7, Rotation2d.fromDegrees(270.0)), 3.0),
            driveToPoseStep(new Pose2d(8.8, 3.6, Rotation2d.fromDegrees(270.0)), 3.0),
            driveToPoseStep(new Pose2d(8.8, 0.7, Rotation2d.fromDegrees(51.56)), 3.0),
            driveToPoseStep(new Pose2d(15.3, 0.8, Rotation2d.fromDegrees(180.0)), 4.0),
            driveToPoseStep(new Pose2d(8.8, 0.7, Rotation2d.fromDegrees(270.0)), 4.0),
            driveToPoseStep(new Pose2d(8.8, 3.6, Rotation2d.fromDegrees(270.0)), 3.0),
            driveToPoseStep(new Pose2d(8.8, 0.7, Rotation2d.fromDegrees(180.0)), 3.0),
            driveToPoseStep(new Pose2d(15.3, 0.7, Rotation2d.fromDegrees(51.56)), 4.0)
        ).withName("Red Left Path");
    }

    private Command createRedRightPathAuto() {
        Pose2d start = new Pose2d(13.0, 7.5, Rotation2d.fromDegrees(180.0));
        return Commands.sequence(
            Commands.runOnce(() -> drivetrain.resetPose(start)),
            driveToPoseStep(new Pose2d(9.0, 7.5, Rotation2d.fromDegrees(90.0)), 3.0),
            driveToPoseStep(new Pose2d(9.0, 4.6, Rotation2d.fromDegrees(90.0)), 3.0),
            driveToPoseStep(new Pose2d(9.0, 7.45, Rotation2d.fromDegrees(180.0)), 3.0),
            driveToPoseStep(new Pose2d(13.2, 7.5, Rotation2d.fromDegrees(180.0)), 3.0),
            driveToPoseStep(new Pose2d(15.0, 6.7, Rotation2d.fromDegrees(180.0)), 3.0),
            driveToPoseStep(new Pose2d(13.0, 7.5, Rotation2d.fromDegrees(180.0)), 3.0),
            driveToPoseStep(new Pose2d(9.0, 7.5, Rotation2d.fromDegrees(90.0)), 3.0),
            driveToPoseStep(new Pose2d(9.0, 4.6, Rotation2d.fromDegrees(90.0)), 3.0),
            driveToPoseStep(new Pose2d(9.0, 7.45, Rotation2d.fromDegrees(180.0)), 3.0),
            driveToPoseStep(new Pose2d(13.2, 7.5, Rotation2d.fromDegrees(180.0)), 3.0),
            driveToPoseStep(new Pose2d(16.25, 7.291, Rotation2d.fromDegrees(180.0)), 4.0)
        ).withName("Red Right Path");
    }
}

