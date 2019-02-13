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
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Arm extends Module {
    private RotatingArmPIDController pid;

    private WPI_TalonSRX leftMotor;
    private WPI_TalonSRX rightMotor;

    private TalonPWMEncoder leftEncoder;

    private Input<Double> holdPoint;

    public Output<Double> pidError;
    public Output<Double> rightEncoderClicks;
    public Output<Double> leftEncoderClicks;
    public Output<Double> redundantEncoderClicks;

    public Arm() {
        super(Arm.class);

        leftMotor = new WPI_TalonSRX(Ports.ARM_LEFT_MOTOR);
        rightMotor = new WPI_TalonSRX(Ports.ARM_RIGHT_MOTOR);

        //For redundency try this.
        //leftMotor = new WPI_TalonSRX(2);
        //rightMotor = new WPI_TalonSRX(3);

        rightMotor.follow(leftMotor);
        rightMotor.setInverted(InvertType.FollowMaster);
        //rightMotor.set(ControlMode.Follower, Ports.ARM_LEFT_MOTOR);

        leftEncoder = new TalonPWMEncoder(leftMotor);

        leftEncoder.zero();

        holdPoint = addInput("Setpoint", 0.0);

		pidError = addOutput("PID Error", () -> pid.getError());
        leftEncoderClicks = addOutput("Right Encoder Clicks", () -> leftEncoder.getPosition());

        this.pid = new RotatingArmPIDController(Calibration.Arm.kP, Calibration.Arm.kI, Calibration.Arm.kD,
            Calibration.Arm.kF, leftEncoder, leftMotor);

        setDefaultAction(move);
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

            //Try commenting out v
            inputPower = addInput("Input Power", 0.0, true);
        }

        @Override
        public void run() {
            //Try commenting out v
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
