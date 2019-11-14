package com._604robotics.robot2019.modules;

import com._604robotics.robot2019.constants.Calibration;
import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.controller.ProfiliedRotatingArmPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import com._604robotics.robotnik.prefabs.devices.RedundantEncoder;
import com._604robotics.robotnik.prefabs.devices.TalonPWMEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ProfiliedArm extends Module {
    private ProfiliedRotatingArmPIDController pid;
    private TrapezoidProfile.Constraints constraints;

    private WPI_TalonSRX leftMotor;
    private WPI_TalonSRX rightMotor;

    private TalonPWMEncoder leftEncoder;
    private RedundantEncoder redundantEncoder;

    public Input<Double> holdPoint;

    public Output<Double> pidError;
    public Output<Double> rightEncoderClicks;
    public Output<Double> leftEncoderClicks;
    public Output<Double> redundantEncoderClicks;

    public ProfiliedArm() {
        super(ProfiliedArm.class);

        leftMotor = new WPI_TalonSRX(Ports.ARM_LEFT_MOTOR);
        rightMotor = new WPI_TalonSRX(Ports.ARM_RIGHT_MOTOR);

        rightMotor.setInverted(true);
        rightMotor.set(ControlMode.Follower, Ports.ARM_LEFT_MOTOR);

    
        leftEncoder = new TalonPWMEncoder(rightMotor);
		leftEncoder.setInverted(false);
        leftEncoder.zero();

        holdPoint = addInput("Setpoint", 300.0);
        leftEncoderClicks = addOutput("Left Encoder Clicks", () -> leftEncoder.getPosition());

        constraints = new TrapezoidProfile.Constraints(4000, 900);

        this.pid = new ProfiliedRotatingArmPIDController(Calibration.Arm.kP, Calibration.Arm.kI, Calibration.Arm.kD,
            Calibration.Arm.kF, constraints);

        pid.setEncoderPeriod(Calibration.Arm.CLICKS_FULL_ROTATION);
        pid.setFeedforwardZeroOffset(Calibration.Arm.HORIZONTAL_POSITION);

        setDefaultAction(move);
    }

    public class Hold extends Action {

        public Hold() {
            super(ProfiliedArm.this, Hold.class);
            holdPoint = addInput("Hold Point", 300.0, true); // TODO low priority move forward
        }

        @Override
        public void run() {
            leftMotor.set(pid.calculate(leftEncoderClicks.get(), holdPoint.get()));
        }

    }

    public class Move extends Action {
        public Input<Double> inputPower;

        public Move() {
            super(ProfiliedArm.this, Move.class);
            inputPower = addInput("Input Power", 0.0, true);
        }

        @Override
        public void run() {
            leftMotor.set(inputPower.get());
        }
    }

    public class Setpoint extends Action {
        public Input<Double> setpoint;

        public Setpoint() {
            super(ProfiliedArm.this, Setpoint.class);
            setpoint = addInput("Setpoint", 0.0, true);
        }

        @Override
        public void run() {
            leftMotor.set(pid.calculate(leftEncoderClicks.get(), setpoint.get()));
        }
    }

    public void resetEncoder() {
        leftEncoder.zero();
    }


    public Hold hold = new Hold();
    public Move move = new Move();
    public Setpoint setpoint = new Setpoint();
}
