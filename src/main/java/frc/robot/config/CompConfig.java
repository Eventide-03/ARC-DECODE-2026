package frc.robot.config;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.config.RobotConfig.SwerveConfig;
import frc.robot.config.RobotConfig.VisionConfig;

class CompConfig {

  public static final RobotConfig competitionBot =
                  new RobotConfig(
                      "comp",
                      new SwerveConfig(new PhoenixPIDController(3.7, 0, 0.0), true, true, true),
                      new VisionConfig(
                          0.005,
                          0.8,
                          // All camera offsets handled in Limelight UI; leave code poses at identity.
                          new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)),
                          new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)),
                          new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)),
                          new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)),
                          new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0))));

  private CompConfig() {}
}
