package com._604robotics.robot2019;

import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robot2019.modes.*;
import com._604robotics.robot2019.modules.*;
import com._604robotics.robotnik.prefabs.modules.*;
import com._604robotics.robot2019.systems.*;
import com._604robotics.robotnik.Robot;
import edu.wpi.first.wpilibj.RobotBase;

public class Robot2019 extends Robot {

    public static void main( String [] args ) {
        RobotBase.startRobot(Robot2019::new);
    }

    public final Dashboard dashboard = addModule(new Dashboard());
    public final DashboardSystem dashboardSystem = addSystem(DashboardSystem.class, new DashboardSystem(this));
    public final PowerMonitor powermonitor = addModule(new PowerMonitor(Ports.PDP_MODULE, Ports.COMPRESSOR));

    public final Drive drive = addModule(new Drive());
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
