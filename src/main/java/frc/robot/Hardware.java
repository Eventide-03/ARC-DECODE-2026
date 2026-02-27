package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightModel;
import frc.robot.vision.limelight.LimelightState;

public class Hardware {
  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController testController = new CommandXboxController(1); 
  public final TalonFX flywheelA1 = new TalonFX(21);
  public final TalonFX flywheelA2 = new TalonFX(22);
  public final TalonFX hopperMotor = new TalonFX(23);
  public final TalonFX hoodMotor = new TalonFX(24);
  public final TalonFX intakeRollerMotor = new TalonFX(25, new CANBus("rio"));
  public final TalonFX intakePivotMotor = new TalonFX(26);
  public final TalonFX indexerMotor = new TalonFX(27);
  public final Limelight leftLimelight = new Limelight("left", LimelightState.TAGS, LimelightModel.FOUR);
  public final Limelight rightLimelight =  new Limelight("right", LimelightState.TAGS, LimelightModel.FOUR);
}
