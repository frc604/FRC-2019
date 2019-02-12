package com._604robotics.robot2019.modules;

import com._604robotics.robot2019.constants.Calibration;
import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.controller.RotatingArmPIDController;
import com._604robotics.robotnik.prefabs.devices.RedundantEncoder;
import com._604robotics.robotnik.prefabs.devices.TalonPWMEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Arm extends Module {
    private RotatingArmPIDController pid;

    private WPI_TalonSRX leftMotor;
    private WPI_TalonSRX rightMotor;

    private TalonPWMEncoder leftEncoder;
    private TalonPWMEncoder rightEncoder;
    private RedundantEncoder redundantEncoder;

    private Input<Double> holdPoint;

    public Output<Double> pidError;
    public Output<Double> rightEncoderClicks;
    public Output<Double> leftEncoderClicks;
    public Output<Double> redundantEncoderClicks;

    public Arm() {
        super(Arm.class);

        leftMotor = new WPI_TalonSRX(Ports.ARM_LEFT_MOTOR);
        rightMotor = new WPI_TalonSRX(Ports.ARM_RIGHT_MOTOR);

        rightMotor.setInverted(true);
        rightMotor.set(ControlMode.Follower, Ports.ARM_LEFT_MOTOR);

        leftEncoder = new TalonPWMEncoder(leftMotor);
        rightEncoder = new TalonPWMEncoder(rightMotor);

        leftEncoder.zero();
        rightEncoder.zero();

        redundantEncoder = new RedundantEncoder(leftEncoder, rightEncoder);
        redundantEncoder.setMinimum(Calibration.Arm.MIN_ENCODER_VAL);
        redundantEncoder.setMaximum(Calibration.Arm.MAX_ENCODER_VAL);

        holdPoint = addInput("Setpoint", 0.0);

		pidError = addOutput("PID Error", () -> pid.getError());
        rightEncoderClicks = addOutput("Right Encoder Clicks", () -> rightEncoder.getPosition());
        leftEncoderClicks = addOutput("Right Encoder Clicks", () -> leftEncoder.getPosition());
        redundantEncoderClicks = addOutput("Redundant Encoder Clicks", () -> redundantEncoder.getValue());

        this.pid = new RotatingArmPIDController(Calibration.Arm.kP, Calibration.Arm.kI, Calibration.Arm.kD,
            Calibration.Arm.kF, leftEncoder, leftMotor);

        setDefaultAction(hold);
    }

    public class Hold extends Action {

        public Hold() {
            super(Arm.this, Hold.class);
            holdPoint = addInput("Hold Point", 0.0, true);
        }

        @Override
        public void begin() {
            pid.setEnabled(true);
        }

        @Override
        public void run() {
            pid.setSetpoint(holdPoint.get());
        }

        @Override
        public void end() {
            pid.setEnabled(false);
        }
    }

    public class Move extends Action {
        public Input<Double> inputPower;

        public Move() {
            super(Arm.this, Move.class);
            inputPower = addInput("Input Power", 0.0, true);
        }

        @Override
        public void run() {
            setpoint.setpoint.set(leftEncoderClicks.get());

            leftMotor.set(inputPower.get());
        }
    }

    public class Setpoint extends Action {
        public Input<Double> setpoint;

        public Setpoint() {
            super(Arm.this, Setpoint.class);
            setpoint = addInput("Setpoint", 0.0, true);
        }

        @Override
        public void begin() {
            pid.setEnabled(true);
        }

        @Override
        public void run() {
            pid.setSetpoint(setpoint.get());
        }

        @Override
        public void end() {
            pid.setEnabled(false);
        }
    }

    public Hold hold = new Hold();
    public Move move = new Move();
    public Setpoint setpoint = new Setpoint();
}
