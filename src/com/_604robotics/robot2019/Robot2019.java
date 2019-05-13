package com._604robotics.robot2019;

import com._604robotics.robot2019.constants.Calibration;
import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robot2019.modes.*;
import com._604robotics.robot2019.modules.*;
import com._604robotics.robotnik.prefabs.modules.*;
import com._604robotics.robot2019.systems.*;
import com._604robotics.robotnik.Robot;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;

public class Robot2019 extends Robot {

    public static void main( String [] args ) {
        RobotBase.startRobot(Robot2019::new);
    }

    public final Dashboard dashboard = addModule(new Dashboard());
    public final DashboardSystem dashboardSystem = addSystem(DashboardSystem.class, new DashboardSystem(this));
    public final PowerMonitor powermonitor = addModule(new PowerMonitor(Ports.PDP_MODULE, Ports.COMPRESSOR));

    public final TankDriveSixWheel drive = addModule(new TankDriveSixWheel(
            new WPI_VictorSPX(Ports.DRIVE_FRONT_LEFT_MOTOR),
            new WPI_VictorSPX(Ports.DRIVE_MIDDLE_LEFT_MOTOR),
            new WPI_VictorSPX(Ports.DRIVE_REAR_LEFT_MOTOR),
            new WPI_VictorSPX(Ports.DRIVE_FRONT_RIGHT_MOTOR),
            new WPI_VictorSPX(Ports.DRIVE_MIDDLE_RIGHT_MOTOR),
            new WPI_VictorSPX(Ports.DRIVE_REAR_RIGHT_MOTOR),
            new Encoder(Ports.ENCODER_LEFT_A,
                    Ports.ENCODER_LEFT_B,
                    false,
                    CounterBase.EncodingType.k4X),
            new Encoder(Ports.ENCODER_RIGHT_A,
                    Ports.ENCODER_RIGHT_B,
                    true,
                    CounterBase.EncodingType.k4X),
            Calibration.DRIVE_MOTOR_RAMP
    ));

    public final Tilter tilter = addModule(new Tilter());

    public final Intake intake = addModule(new Intake());
    public final Arm arm = addModule(new Arm());
    public final Hardstop hardstop = addModule(new Hardstop());

    public final HatchSlider slider = addModule(new HatchSlider());
    public final Hook hook = addModule(new Hook());

    public final Limelight limelight = addModule(new Limelight());
	
	public final Camera frontCamera = addModule(new Camera("video0"));
	public final Camera backCamera = addModule(new Camera("video1"));

    public final TeleopMode teleopMode = setTeleopMode(new TeleopMode(this));
    public final AutonomousMode autonomousMode = setAutonomousMode(new AutonomousMode(this));
}
