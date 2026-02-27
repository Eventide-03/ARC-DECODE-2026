package frc.robot.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;


public class IntakePosition extends StateMachine<IntakePosition.State> {
	public enum State { OFF, UP, DOWN }

	private static final double GEAR_RATIO = 75.0; 

	private final TalonFX motor;
	private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withSlot(0);

	private double upDegrees = 0.0;
	private double downDegrees = 35.0;


	public IntakePosition(TalonFX motor) {
		super(SubsystemPriority.DEPLOY, State.OFF);
		this.motor = motor;

		var cfg = new TalonFXConfiguration();
		cfg.Slot0 = new Slot0Configs().withKP(80).withKI(0).withKD(0);
		cfg.MotionMagic = new MotionMagicConfigs()
				.withMotionMagicCruiseVelocity(15.0)
				.withMotionMagicAcceleration(30.0);
		cfg.CurrentLimits = new CurrentLimitsConfigs()
				.withSupplyCurrentLimit(45)
				.withSupplyCurrentLimitEnable(true)
				.withStatorCurrentLimit(50)
				.withStatorCurrentLimitEnable(true);
		motor.getConfigurator().apply(cfg);
	}

	public void setUpDegrees(double deg) { this.upDegrees = deg; }
	public void setDownDegrees(double deg) { this.downDegrees = deg; }

	public void requestUp() { setStateFromRequest(State.UP); }
	public void requestDown() { setStateFromRequest(State.DOWN); }
	public void off() { setStateFromRequest(State.OFF); }

	private static double degreesToRotations(double deg) {
		return (deg / 360.0) * GEAR_RATIO;
	}

	private void goToDegrees(double deg) {
		motor.setControl(mmRequest.withPosition(degreesToRotations(deg)));
	}

	@Override
	protected State getNextState(State current) { return current; }

	@Override
	protected void afterTransition(State newState) {
		switch (newState) {
			case OFF -> {
				double currentRot = motor.getPosition().getValueAsDouble();
				motor.setControl(mmRequest.withPosition(currentRot));
			}
			case UP -> goToDegrees(upDegrees);
			case DOWN -> goToDegrees(downDegrees);
		}
	}
}
