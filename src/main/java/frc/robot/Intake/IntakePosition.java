package frc.robot.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.IndexerSubsystem.Hopper;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class IntakePosition extends StateMachine<IntakePosition.State> {
	public enum State {
		OFF,
		DEPLOY,
		RETRACT
	}

	private static final double DEPLOY_ROTATIONS = -4;

	private final TalonFX motor;
	private final MotionMagicTorqueCurrentFOC mmRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(0);

	public IntakePosition(TalonFX motor) {
		super(SubsystemPriority.DEPLOY, State.OFF);
		this.motor = motor;
		

		var cfg = new TalonFXConfiguration();
cfg.Slot0 = new Slot0Configs()
    .withKP(30).withKI(0).withKD(0)
    .withKG(0.3)
    .withKS(0.1);
cfg.MotionMagic = new MotionMagicConfigs()
    .withMotionMagicCruiseVelocity(150.0)
    .withMotionMagicAcceleration(160.0);
cfg.CurrentLimits = new CurrentLimitsConfigs()
    .withSupplyCurrentLimit(60)
    .withSupplyCurrentLimitEnable(true)
    .withStatorCurrentLimit(160)
    .withStatorCurrentLimitEnable(true);
	motor.getConfigurator().apply(cfg);
		motor.setPosition(0.0);	
}

	/** Move elevator to deployed position. */
	public void deploy()  { setStateFromRequest(State.DEPLOY); }
	/** Retract elevator back to boot/zero position. */
	public void retract() { setStateFromRequest(State.RETRACT); }

	@Override
	protected State getNextState(State current) { return current; }

	@Override
	protected void afterTransition(State newState) {
		switch (newState) {
			case OFF     -> motor.setControl(new CoastOut());
			case DEPLOY  -> motor.setControl(mmRequest.withPosition(DEPLOY_ROTATIONS));
			case RETRACT -> motor.setControl(mmRequest.withPosition(-0.85));
		}
	}
}
