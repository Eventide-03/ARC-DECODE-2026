package frc.robot.controller;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.AutoMovements.HeadingLock;
import frc.robot.FlywheelSubsystem.Flywheel;
import frc.robot.FlywheelSubsystem.FlywheelStateMachine;
import frc.robot.FlywheelSubsystem.Hood;
import frc.robot.FlywheelSubsystem.HoodStateMachine;
import frc.robot.IndexerSubsystem.Indexer;
import frc.robot.IndexerSubsystem.Hopper;
import frc.robot.Intake.IntakePosition;

/*
 
controls
dpad up/down  hood target +-1 deg; report current angle.
dpad right/left flywheel target +-50 rpm report target.
right bumper  -toggle heading lock enable/disable.
b/x - Turret offset +-1 deg report current offset.
left/right trigger - intakepos up setpoint +-1 deg; also adjust indexer/hopper targets by +-50 rpm conv to dut
a - toggle intakeposition between Up and Down (starts Up).
 */
public class TestController {
  private final CommandXboxController controller;
  private final Hood hood;
  private final HoodStateMachine hoodSM;
  private final Flywheel flywheel;
  private final FlywheelStateMachine flywheelSM;
  private final HeadingLock headingLock;
  private final IntakePosition intakePosition;
  private final Indexer indexer;
  private final Hopper hopper;

  private double hoodTargetDeg;
  private double flywheelTargetRpm;
  private boolean headingLockEnabled = false;
  private double turretOffsetDeg = 0.0;
  private double intakeUpDeg = 0.0;
  private boolean intakeIsUp = true;

  private double indexerTargetRpm = 0.0;
  private double hopperTargetRpm = 0.0;

  private static final double RPM_TO_DUTY_SCALE = 5000.0; 

  public TestController(CommandXboxController controller,
      Hood hood,
      HoodStateMachine hoodSM,
      Flywheel flywheel,
      FlywheelStateMachine flywheelSM,
      HeadingLock headingLock,
      IntakePosition intakePosition,
      Indexer indexer,
      Hopper hopper) {
    this.controller = controller;
    this.hood = hood;
    this.hoodSM = hoodSM;
    this.flywheel = flywheel;
    this.flywheelSM = flywheelSM;
    this.headingLock = headingLock;
    this.intakePosition = intakePosition;
    this.indexer = indexer;
    this.hopper = hopper;

    this.hoodTargetDeg = safeHoodDeg();
    this.flywheelTargetRpm = 0.0;
    this.intakeUpDeg = 0.0; // def
    this.intakeIsUp = true;
    this.indexerTargetRpm = 0.0;
    this.hopperTargetRpm = 0.0;

    intakePosition.requestUp();

    bind();
    publishAll();
  }

  private void bind() {
    controller.povUp().onTrue(
      edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
        hoodTargetDeg += 1.0;
        hoodSM.requestDegrees(hoodTargetDeg);
        SmartDashboard.putNumber("Test/Hood/target_deg", hoodTargetDeg);
        SmartDashboard.putNumber("Test/Hood/current_deg", safeHoodDeg());
      })
    );

    controller.povDown().onTrue(
      edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
        hoodTargetDeg -= 1.0;
        hoodSM.requestDegrees(hoodTargetDeg);
        SmartDashboard.putNumber("Test/Hood/target_deg", hoodTargetDeg);
        SmartDashboard.putNumber("Test/Hood/current_deg", safeHoodDeg());
      })
    );

    controller.povRight().onTrue(
      edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
        flywheelTargetRpm += 50.0;
        flywheelSM.requestRpm(flywheelTargetRpm);
        SmartDashboard.putNumber("Test/Flywheel/target_rpm", flywheelTargetRpm);
      })
    );

    controller.povLeft().onTrue(
      edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
        flywheelTargetRpm = Math.max(0.0, flywheelTargetRpm - 50.0);
        flywheelSM.requestRpm(flywheelTargetRpm);
        SmartDashboard.putNumber("Test/Flywheel/target_rpm", flywheelTargetRpm);
      })
    );

    controller.rightBumper().onTrue(
      edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
        headingLockEnabled = !headingLockEnabled;
        if (headingLockEnabled) {
          headingLock.enableForAlliance();
        } else {
          headingLock.disableLock();
        }
        SmartDashboard.putBoolean("Test/HeadingLock/enabled", headingLockEnabled);
      })
    );

    controller.b().onTrue(
      edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
        turretOffsetDeg += 1.0;
        headingLock.setTurretOffsetDegrees(turretOffsetDeg);
        SmartDashboard.putNumber("Test/Turret/offset_deg", turretOffsetDeg);
      })
    );

    controller.x().onTrue(
      edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
        turretOffsetDeg -= 1.0;
        headingLock.setTurretOffsetDegrees(turretOffsetDeg);
        SmartDashboard.putNumber("Test/Turret/offset_deg", turretOffsetDeg);
      })
    );

    controller.leftTrigger().onTrue(
      edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
        intakeUpDeg += 1.0;
        intakePosition.setUpDegrees(intakeUpDeg);
        if (intakeIsUp) {
          intakePosition.requestUp();
        }
        SmartDashboard.putNumber("Test/Intake/up_deg", intakeUpDeg);
        SmartDashboard.putBoolean("Test/Intake/is_up", intakeIsUp);

        indexerTargetRpm = Math.max(0.0, indexerTargetRpm - 50.0);
        double idxDuty = clamp(indexerTargetRpm / RPM_TO_DUTY_SCALE);
        indexer.setDutyPercent(idxDuty);
        SmartDashboard.putNumber("Test/Indexer/target_rpm", indexerTargetRpm);
        SmartDashboard.putNumber("Test/Indexer/duty", idxDuty);
      })
    );

    controller.rightTrigger().onTrue(
      edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
        intakeUpDeg = Math.max(0.0, intakeUpDeg - 1.0);
        intakePosition.setUpDegrees(intakeUpDeg);
        if (intakeIsUp) {
          intakePosition.requestUp();
        }
        SmartDashboard.putNumber("Test/Intake/up_deg", intakeUpDeg);
        SmartDashboard.putBoolean("Test/Intake/is_up", intakeIsUp);

        hopperTargetRpm += 50.0;
        indexerTargetRpm += 50.0;
        double hopDuty = clamp(hopperTargetRpm / RPM_TO_DUTY_SCALE);
        double idxDuty = clamp(indexerTargetRpm / RPM_TO_DUTY_SCALE);
        hopper.setDutyPercent(hopDuty);
        indexer.setDutyPercent(idxDuty);
        SmartDashboard.putNumber("Test/Hopper/target_rpm", hopperTargetRpm);
        SmartDashboard.putNumber("Test/Hopper/duty", hopDuty);
        SmartDashboard.putNumber("Test/Indexer/target_rpm", indexerTargetRpm);
        SmartDashboard.putNumber("Test/Indexer/duty", idxDuty);
      })
    );

    controller.a().onTrue(
      edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
        intakeIsUp = !intakeIsUp;
        if (intakeIsUp) {
          intakePosition.requestUp();
        } else {
          intakePosition.requestDown();
        }
        SmartDashboard.putBoolean("Test/Intake/is_up", intakeIsUp);
      })
    );
  }

  private void publishAll() {
    SmartDashboard.putNumber("Test/Hood/target_deg", hoodTargetDeg);
    SmartDashboard.putNumber("Test/Hood/current_deg", safeHoodDeg());
    SmartDashboard.putNumber("Test/Flywheel/target_rpm", flywheelTargetRpm);
    SmartDashboard.putBoolean("Test/HeadingLock/enabled", headingLockEnabled);
    SmartDashboard.putNumber("Test/Turret/offset_deg", turretOffsetDeg);
    SmartDashboard.putNumber("Test/Intake/up_deg", intakeUpDeg);
    SmartDashboard.putBoolean("Test/Intake/is_up", intakeIsUp);
    SmartDashboard.putNumber("Test/Indexer/target_rpm", indexerTargetRpm);
    SmartDashboard.putNumber("Test/Indexer/duty", clamp(indexerTargetRpm / RPM_TO_DUTY_SCALE));
    SmartDashboard.putNumber("Test/Hopper/target_rpm", hopperTargetRpm);
    SmartDashboard.putNumber("Test/Hopper/duty", clamp(hopperTargetRpm / RPM_TO_DUTY_SCALE));
  }

  private double safeHoodDeg() {
    try {
      return hood.getAngleDegrees();
    } catch (Exception e) {
      return hoodTargetDeg;
    }
  }

  private static double clamp(double v) {
    if (v > 1.0) return 1.0;
    if (v < -1.0) return -1.0;
    return v;
  }
}
