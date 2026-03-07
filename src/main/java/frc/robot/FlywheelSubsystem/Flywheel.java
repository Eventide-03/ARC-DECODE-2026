package frc.robot.FlywheelSubsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Flywheel {
	private final TalonFX a1, a2;

	private final VelocityTorqueCurrentFOC velocityRequest =
			new VelocityTorqueCurrentFOC(0).withSlot(0);


	public static final double kV = 0.325;
	public static final double kP = 3.6;

	
		public static final double SUPPLY_LIMIT = 60.0;
	public static final double STATOR_LIMIT = 80.0;
	public static final double TORQUE_CURRENT_LIMIT = 80.0;

	public static final double RPM_TOLERANCE = 400;
	public static final double AT_GOAL_DEBOUNCE_TIME = 0.06;

	// ── State ────────────────────────────────────────────────────
	private final Debouncer atGoalDebounce =
			new Debouncer(AT_GOAL_DEBOUNCE_TIME, Debouncer.DebounceType.kFalling);

	private double targetRpm = 0.0;
	private boolean atGoal   = false;
	private double liveKP = kP;
	private double liveKV = kV;

	public Flywheel(TalonFX a1, TalonFX a2) {
		this.a1 = a1;
		this.a2 = a2;

		var cfg = new TalonFXConfiguration();

		cfg.Slot0 = new Slot0Configs()
				.withKV(kV)
				.withKP(kP);

	
		cfg.CurrentLimits = new CurrentLimitsConfigs()
				.withSupplyCurrentLimit(SUPPLY_LIMIT)
				.withSupplyCurrentLimitEnable(true)
				.withStatorCurrentLimit(STATOR_LIMIT)
				.withStatorCurrentLimitEnable(true);

		// --- Torque-current limits (used by FOC control modes) ---
		cfg.TorqueCurrent = new TorqueCurrentConfigs()
				.withPeakForwardTorqueCurrent(TORQUE_CURRENT_LIMIT)
				.withPeakReverseTorqueCurrent(-TORQUE_CURRENT_LIMIT);

		a1.getConfigurator().apply(cfg);
		a2.getConfigurator().apply(cfg);

		// Publish initial tuning values for live adjustment
		SmartDashboard.putNumber("Flywheel/Tuning/kP", kP);
		SmartDashboard.putNumber("Flywheel/Tuning/kV", kV);
	}

	/**
	 * Command the flywheel to a target RPM using direct velocity FOC.
	 *
	 * No motion profile — the controller sees the full RPM error instantly,
	 * so recovery after a ball launch or disturbance is as fast as physics allows.
	 * kV feedforward carries the steady-state load, kP snaps the error to zero,
	 * and kD damps any ringing before it starts.
	 */
	public void spinFlywheel(double rpm) {
		targetRpm = Math.max(0.0, rpm);

		double measured = getRpm();
		double error    = targetRpm - measured;
		boolean inTol   = Math.abs(error) <= RPM_TOLERANCE;
		atGoal = atGoalDebounce.calculate(inTol);

		if (targetRpm <= 1e-3) {
			stop();
			return;
		}

		// Convert RPM → rotations per second (Phoenix 6 native velocity unit)
		double targetRps = targetRpm / 60.0;
		var request = velocityRequest.withVelocity(targetRps);

		a1.setControl(request);
		a2.setControl(request);
	}

	public void stop() {
		targetRpm = 0.0;
		atGoal = false;
		a1.setControl(new NeutralOut());
		a2.setControl(new NeutralOut());
	}

	public double getRpm() {
		double v1 = safeVel(a1);
		double v2 = safeVel(a2);
		double rpsAvg = (Math.abs(v1) + Math.abs(v2)) / 2.0;
		return rpsAvg * 60.0;
	}

	public boolean isAtGoal() { return atGoal; }

	private static double safeVel(TalonFX fx) {
		try {
			return fx.getRotorVelocity().getValueAsDouble();
		} catch (Exception ex) {
			return 0.0;
		}
	}


	public void periodicTelemetry() {
		SmartDashboard.putNumber("Flywheel/ActualRPM", getRpm());
		SmartDashboard.putNumber("Flywheel/TargetRPM", targetRpm);
		SmartDashboard.putBoolean("Flywheel/AtGoal", atGoal);
		SmartDashboard.putNumber("Flywheel/ErrorRPM", targetRpm - getRpm());

		// Live tuning from SmartDashboard
		double newKP = SmartDashboard.getNumber("Flywheel/Tuning/kP", liveKP);
		double newKV = SmartDashboard.getNumber("Flywheel/Tuning/kV", liveKV);

		if (newKP != liveKP || newKV != liveKV) {
			liveKP = newKP;
			liveKV = newKV;

			var newSlot0 = new Slot0Configs()
					.withKV(liveKV)
					.withKP(liveKP);
			a1.getConfigurator().apply(newSlot0);
			a2.getConfigurator().apply(newSlot0);
		}
	}
}
