package frc.robot.FlywheelSubsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Hood {
	private static final double GEAR_RATIO = 84.0; 
	private static final double MIN_DEG = 15.0;
	private static final double MAX_DEG = 45.0;
	private static final double AT_GOAL_TOL_DEG = 0.1;

	private final TalonFX motor;
	private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withSlot(0);
	private double lastTargetDeg = 0.0;

	public Hood(TalonFX motor) {
		this.motor = motor;

		var cfg = new TalonFXConfiguration();
		cfg.Slot0 = new Slot0Configs().withKP(80).withKI(0).withKD(0);
		cfg.MotionMagic =
				new MotionMagicConfigs()
						.withMotionMagicCruiseVelocity(20.0)
						.withMotionMagicAcceleration(40.0);
		cfg.CurrentLimits = new CurrentLimitsConfigs()
				.withSupplyCurrentLimit(20.0)
				.withSupplyCurrentLimitEnable(true)
				.withStatorCurrentLimit(40.0)
				.withStatorCurrentLimitEnable(true);

		motor.getConfigurator().apply(cfg);
	}

	public void setAngleDegrees(double degrees) {
		double clamped = Math.max(MIN_DEG, Math.min(MAX_DEG, degrees));
		lastTargetDeg = clamped;
		double rotations = (clamped / 360.0) * GEAR_RATIO;
		motor.setControl(mmRequest.withPosition(rotations));
	}

	public void hold() {
		double currentRot = motor.getPosition().getValueAsDouble();
		motor.setControl(mmRequest.withPosition(currentRot));
	}

	public double getAngleDegrees() {
		double rotations = motor.getPosition().getValueAsDouble();
		return (rotations / GEAR_RATIO) * 360.0;
	}

	public boolean isAtGoal() {
		return Math.abs(getAngleDegrees() - lastTargetDeg) <= AT_GOAL_TOL_DEG;
	}

	public void periodicTelemetry() {
		SmartDashboard.putNumber("Hood/AngleDeg", getAngleDegrees());
		SmartDashboard.putNumber("Hood/TargetDeg", lastTargetDeg);
		SmartDashboard.putBoolean("Hood/AtGoal", isAtGoal());
	}

	public double getAngleRadians() {
		return Math.toRadians(getAngleDegrees());
	}
}
