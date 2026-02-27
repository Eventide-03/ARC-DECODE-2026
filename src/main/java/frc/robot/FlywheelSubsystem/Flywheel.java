package frc.robot.FlywheelSubsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;


public class Flywheel {
	private final TalonFX a1, a2;

	private static final double RPM_TOLERANCE = 75.0; 
	private static final double DUTY_HIGH = 1.0;       
	private static final double DUTY_HOLD = 0.0;       

	private final Debouncer atGoalDebounce = new Debouncer(0.2, Debouncer.DebounceType.kFalling);

	private double targetRpm = 0.0;
	private boolean atGoal = false;

	public Flywheel(TalonFX a1, TalonFX a2) {
		this.a1 = a1;
		this.a2 = a2;

		var cfg = new TalonFXConfiguration();
		cfg.CurrentLimits = new CurrentLimitsConfigs()
				.withSupplyCurrentLimit(60.0)
				.withSupplyCurrentLimitEnable(true)
				.withStatorCurrentLimit(80.0)
				.withStatorCurrentLimitEnable(true);
		a1.getConfigurator().apply(cfg);
		a2.getConfigurator().apply(cfg);
	}

	public void spinFlywheel(double rpm) {
		targetRpm = Math.max(0.0, rpm);

		double measured = getRpm();
		boolean inTol = Math.abs(measured - targetRpm) <= RPM_TOLERANCE;
		atGoal = atGoalDebounce.calculate(inTol);

		if (targetRpm <= 1e-3) {
			setDuty(0.0);
			return;
		}

		double duty = (measured + RPM_TOLERANCE < targetRpm) ? DUTY_HIGH : DUTY_HOLD;
		setDuty(duty);
	}

	public void stop() {
		targetRpm = 0.0;
		setDuty(0.0);
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

	private void setDuty(double percent) {
		a1.setControl(new DutyCycleOut(percent));
		a2.setControl(new DutyCycleOut(-percent));
	}
}
