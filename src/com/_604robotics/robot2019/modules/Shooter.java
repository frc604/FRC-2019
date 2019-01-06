package com._604robotics.robot2019.modules;

import com._604robotics.robot2019.constants.Calibration;
import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.devices.TalonPWMEncoder;
import com._604robotics.robotnik.prefabs.devices.wrappers.MultiOutput;
import com._604robotics.robotnik.prefabs.flow.Pulse;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

public class Shooter extends Module {

	private Pulse shooterReady = new Pulse();
	
	private final Victor wheelMid = new Victor(Ports.WHEEL_MID);

	private final WPI_TalonSRX wheel_top_a = new WPI_TalonSRX(Ports.WHEEL_TOP_A);
	private final WPI_TalonSRX wheel_top_b = new WPI_TalonSRX(Ports.WHEEL_TOP_B);
	
	private final MultiOutput wheel_top = new MultiOutput(wheel_top_a, wheel_top_b);
	private final MultiOutput wheel_mid = new MultiOutput(wheelMid);

	private final TalonPWMEncoder encoder_top = new TalonPWMEncoder(wheel_top_a);

	public final Output<Double> topRate = addOutput("Top Rate", encoder_top::getVelocity);
	public final Output<Boolean> isCharged = addOutput("Is Charged", shooterReady::isHigh);

	private final Timer chargeTimer = new Timer();

	public final Action idle = new Idle();
	public final Action shooterStartup = new ShooterStartup();

	public class Idle extends Action {
		public Idle() {
			super(Shooter.this, Idle.class);
		}

		@Override
		public void run() {
			wheel_top.stopMotor();
			wheel_mid.stopMotor();
		}
	}

	public class ShooterStartup extends Action {
		public final Input<Double> motorSpeed;
		public final Input<Boolean> maximumOverdrive;

		public ShooterStartup() {
			this(0, false);
		}
		
		public ShooterStartup(double motorSpeed) {
			this(motorSpeed, false);
		}
		
		public ShooterStartup(boolean maximumOverdrive) {
			this(0, maximumOverdrive);
		}

		public ShooterStartup(double motorSpeed, boolean maximumOverdrive) {
			super(Shooter.this, ShooterStartup.class);
			this.motorSpeed = addInput("motorSpeed", motorSpeed, true);
			this.maximumOverdrive = addInput("maximumOverdrive", maximumOverdrive, true);
		}

		@Override
		public void begin() {
			chargeTimer.start();
			shooterReady.update(false);
		}

		@Override
		public void run() {
			if (Math.abs(Calibration.SHOOTER_TOP_RATE_TARGET
					- encoder_top.getVelocity()) >= Calibration.SHOOTER_TOP_RATE_THRESHOLD) {
				chargeTimer.reset();
				shooterReady.update(false);
			}

			if (encoder_top.getVelocity() >= Calibration.SHOOTER_TOP_RATE_TARGET + Calibration.SHOOTER_TOP_RATE_THRESHOLD) {
				wheel_top.stopMotor();
				wheel_mid.stopMotor();
			} else {
				double speed = motorSpeed.get()*Calibration.TOP_WHEEL_SPEED_LIMITER;
				if( maximumOverdrive.get() && motorSpeed.get() > 0 ) {
					speed *= 10/7;
				}
				wheel_top.set(speed);
				wheel_mid.set(speed);
			}

			if (chargeTimer.get() >= Calibration.MIN_CHARGE_TIME) {
				shooterReady.update(true);
			}
		}

		@Override
		public void end() {
			chargeTimer.stop();
			chargeTimer.reset();
			wheel_top.stopMotor();
			wheel_mid.stopMotor();
			shooterReady.update(false);
		}
	}

	// `synchronized` not quite necessary without PIDControllers
	// Leaving this here in case switch to PID is necessary

	public Shooter() {
		super(Shooter.class);
		setDefaultAction(idle);
		wheel_top_a.setInverted(true);
		wheelMid.setInverted(true);
	}
}