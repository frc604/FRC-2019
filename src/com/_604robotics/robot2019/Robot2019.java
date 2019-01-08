package com._604robotics.robot2019;

import com._604robotics.robot2019.constants.Ports;
import com._604robotics.robot2019.modes.*;
import com._604robotics.robot2019.modules.*;
import com._604robotics.robotnik.prefabs.devices.Limelight;
import com._604robotics.robotnik.prefabs.devices.pixy.PixyI2C;
import com._604robotics.robotnik.prefabs.modules.*;
import com._604robotics.robot2019.systems.*;
import com._604robotics.robotnik.Robot;

public class Robot2019 extends Robot {
    public final Dashboard dashboard = addModule(new Dashboard());
    public final DashboardSystem dashboardSystem = addSystem(DashboardSystem.class, new DashboardSystem(this));
    public final PowerMonitor powermonitor = addModule(new PowerMonitor(Ports.PDP_MODULE, Ports.COMPRESSOR));

    public final Drive drive = addModule(new Drive());
    public final Shifter shifter = addModule(new Shifter( Ports.SHIFTER_A, Ports.SHIFTER_B));

    public final Camera cameraFront = addModule(new Camera("video0"));
    public final Camera cameraBack = addModule(new Camera("video1"));

    public final TeleopMode teleopMode = setTeleopMode(new TeleopMode(this));
    public final AutonomousMode autonomousMode = setAutonomousMode(new AutonomousMode(this));
}
