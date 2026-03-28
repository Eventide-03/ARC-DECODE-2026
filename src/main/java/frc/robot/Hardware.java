package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightModel;
import frc.robot.vision.limelight.LimelightState;

public class Hardware {

  private static final CANBus RIO = new CANBus("rio");

  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController testController   = new CommandXboxController(1); // testing only

  // Requested CAN IDs
  public final TalonFX flywheelA1        = new TalonFX(4, RIO);   // shooter bottom
  public final TalonFX flywheelA2        = new TalonFX(6, RIO);   // shooter top spin
  public final TalonFX hopperMotor       = new TalonFX(3, RIO);
  public final TalonFX hoodMotor         = new TalonFX(5, RIO);   // shooter hood
  public final TalonFX indexerMotor      = new TalonFX(7, RIO);
  public final TalonFX intakePivotMotor  = new TalonFX(2, RIO);
  public final TalonFX intakeRollerMotor = new TalonFX(16, RIO);

  public final Limelight leftLimelight  = new Limelight("left",  LimelightState.TAGS, LimelightModel.FOUR);
  public final Limelight rightLimelight = new Limelight("right", LimelightState.TAGS, LimelightModel.FOUR);
}
