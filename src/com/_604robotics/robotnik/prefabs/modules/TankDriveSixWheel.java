package com._604robotics.robotnik.prefabs.modules;

import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.devices.wrappers.RampMotor;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

public class TankDriveSixWheel extends Module {
    private final RampMotor m_frontLeft;
    private final RampMotor m_middleLeft;
    private final RampMotor m_rearLeft;
    private final SpeedControllerGroup m_left;

    private final RampMotor m_frontRight;
    private final RampMotor m_middleRight;
    private final RampMotor m_rearRight;
    private final SpeedControllerGroup m_right;

    DifferentialDrive robotDrive;

    private final Encoder encoderLeft;
    private final Encoder encoderRight;
    
    private final Accelerometer accel;
    public final Output<Double> xAccel;
    public final Output<Double> yAccel;
    public final Output<Double> zAccel;

    public synchronized void resetSensors() {
        encoderLeft.reset();
        encoderRight.reset();
    }

    public final Output<Integer> leftClicks;
    public final Output<Integer> rightClicks;
    
    public final Output<Double> leftClickRate;
    public final Output<Double> rightClickRate;

    public class Idle extends Action {
        public Idle () {
            super(TankDriveSixWheel.this, Idle.class);
        }

        @Override
        public void run () {
            robotDrive.stopMotor();
        }
    }

    public final Action idle = new Idle();

    public class TankDrive extends Action {
        public final Input<Double> leftPower;
        public final Input<Double> rightPower;
        public final boolean squared;

        public TankDrive () {
            this(0, 0, true);
        }
        
        public TankDrive (boolean squared) {
            this(0, 0, squared);
        }

        public TankDrive (double defaultLeftPower, double defaultRightPower) {
            this(defaultLeftPower, defaultRightPower, true);
        }

        public TankDrive (double defaultLeftPower, double defaultRightPower, boolean squared) {
            super(TankDriveSixWheel.this, TankDrive.class);
            // Make these inputs persistent for auton code
            leftPower = addInput("leftPower", defaultLeftPower, true);
            rightPower = addInput("rightPower", defaultRightPower, true);
            this.squared = squared; 
        }

        @Override
        public void run () {
            if (leftPower.get()>1 || leftPower.get()<-1 || rightPower.get()>1 || rightPower.get()<-1) {
                System.out.println("L"+leftPower.get()+"R"+rightPower.get());
            }
            robotDrive.tankDrive(leftPower.get(), rightPower.get(), squared);
        }
    }

    public class ArcadeDrive extends Action {
        public final Input<Double> movePower;
        public final Input<Double> rotatePower;
        public final boolean squared;

        public ArcadeDrive () {
            this(0, 0, true);
        }
        
        public ArcadeDrive (boolean squared) {
            this(0, 0, squared);
        }

        public ArcadeDrive (double defaultMovePower, double defaultRotPower) {
            this(defaultMovePower, defaultRotPower, true);
        }

        public ArcadeDrive (double defaultMovePower, double defaultRotPower, boolean squared) {
            super(TankDriveSixWheel.this, ArcadeDrive.class);
            // Make these inputs persistent for auton code
            movePower = addInput("movePower", defaultMovePower, true);
            rotatePower = addInput("rotatePower", defaultRotPower, true);
            this.squared = squared;
        }

        @Override
        public void run () {
            robotDrive.arcadeDrive(movePower.get(), rotatePower.get(), squared);
        }
    }
    
    public TankDriveSixWheel( SpeedController frontLeft, SpeedController frontRight, SpeedController middleLeft,
                              SpeedController middleRight, SpeedController rearLeft, SpeedController rearRight,
                              Encoder leftEncoder, Encoder rightEncoder, Accelerometer accel, double driveMotorRamp ) {
        super(TankDriveSixWheel.class);

        m_frontLeft = new RampMotor(frontLeft, driveMotorRamp);
        m_middleLeft =  new RampMotor(middleLeft, driveMotorRamp);
        m_rearLeft = new RampMotor(rearLeft, driveMotorRamp);
        m_frontRight = new RampMotor(frontRight, driveMotorRamp);
        m_middleRight = new RampMotor(middleRight, driveMotorRamp);
        m_rearRight = new RampMotor(rearRight, driveMotorRamp);

        m_left = new SpeedControllerGroup(m_frontLeft, m_middleLeft, m_rearLeft);
        m_right = new SpeedControllerGroup(m_frontRight, m_middleRight, m_rearRight);

        robotDrive = new DifferentialDrive(m_left, m_right);

        this.encoderLeft = leftEncoder;
        this.encoderRight = rightEncoder;

        this.accel = accel;

        xAccel = addOutput("X accel",accel::getX);
        yAccel = addOutput("Y accel",accel::getY);
        zAccel = addOutput("Z accel",accel::getZ);

        leftClicks = addOutput("leftClicks", encoderLeft::get);
        rightClicks = addOutput("rightClicks", encoderRight::get);
        leftClickRate = addOutput("leftClickRate", encoderLeft::getRate);
        rightClickRate = addOutput("rightClickRate", encoderRight::getRate);

        robotDrive.setDeadband(0.04);
        setDefaultAction(idle);
    }

    public TankDriveSixWheel( SpeedController frontLeft, SpeedController frontRight, SpeedController middleLeft,
                              SpeedController middleRight, SpeedController rearLeft, SpeedController rearRight,
                              Encoder leftEncoder, Encoder rightEncoder, double driveMotorRamp ) {
        this(frontLeft, frontRight, middleLeft, middleRight, rearLeft, rearRight,
                leftEncoder, rightEncoder, new BuiltInAccelerometer(), driveMotorRamp);
    }
}
