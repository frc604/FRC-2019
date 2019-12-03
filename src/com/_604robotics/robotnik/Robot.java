package com._604robotics.robotnik;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Base64;
import java.util.List;
import java.util.stream.Collectors;
import java.util.zip.InflaterInputStream;

import com._604robotics.robotnik.utils.Pair;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    private static final Logger logger = new Logger(Robot.class);
    
    private final NetworkTableInstance network = NetworkTableInstance.getDefault();
    private final NetworkTable table = network.getTable("robotnik");

    private final List<Module> modules = new ArrayList<>();

    private final List<Pair<String, Coordinator>> systems = new ArrayList<>();

    private Coordinator autonomousMode;
    private Coordinator teleopMode;
    private Coordinator testMode;

    private Coordinator currentMode = null;
    private String currentModeName = null;

    public Robot () {
        updateModuleList();
    }

    protected <T extends Module> T addModule (T module) {
        modules.add(module);
        updateModuleList();
        return module;
    }

    private void updateModuleList () {
    	table.getEntry("moduleList").setString( modules.stream().map( Module::getName ).collect( Collectors.joining(",") ) );
    }

    protected <T extends Coordinator> T addSystem (String name, T system) {
        systems.add(new Pair<>(name, system));
        return system;
    }

    protected <T extends Coordinator> T addSystem (Class<T> klass, T system) {
        addSystem(klass.getSimpleName(), system);
        return system;
    }

    protected <T extends Coordinator> T setAutonomousMode (T autonomousMode) {
        this.autonomousMode = autonomousMode;
        return autonomousMode;
    }

    protected <T extends Coordinator> T setTeleopMode (T teleopMode) {
        this.teleopMode = teleopMode;
        return teleopMode;
    }

    protected <T extends Coordinator> T setTestMode (T testMode) {
        this.testMode = testMode;
        return testMode;
    }


    private void loopInit(String name, Coordinator mode) {
        logger.info(name + " mode begin");

        currentMode = mode;
        currentModeName = name;

        for (Module module : modules) {
            Reliability.swallowThrowables(module::begin, "Error in begin() of module " + module.getName());
        }

        if (mode != null) {
            Reliability.swallowThrowables(mode::start, "Error starting " + name + " mode");
        }

        for (Pair<String, Coordinator> system : systems) {
            Reliability.swallowThrowables(system.getValue()::start, "Error starting system " + system.getKey());
        }
    }

    private void loopExecute(String name, Coordinator mode) {
        for (Module module : modules) {
            module.prepare();
        }

        if (mode != null) {
            Reliability.swallowThrowables(mode::execute, "Error executing " + name + " mode");
        }

        for (Pair<String, Coordinator> system : systems) {
            Reliability.swallowThrowables(system.getValue()::execute, "Error executing system " + system.getKey());
        }

        for (Module module : modules) {
            module.update();

            if (mode != null) {
                module.execute();
            }

            Reliability.swallowThrowables(module::run, "Error in run() of module " + module.getName());
        }

    }

    private void loopEnd(String name, Coordinator mode) {
        if (mode != null) {
            Reliability.swallowThrowables(mode::stop, "Error stopping " + name + " mode");
        }

        for (Pair<String, Coordinator> system : systems) {
            Reliability.swallowThrowables(system.getValue()::stop, "Error stopping system " + system.getKey());
        }

        for (Module module : modules) {
            if (mode != null) {
                module.terminate();
            }

            Reliability.swallowThrowables(module::end, "Error in end() of module " + module.getName());
        }
    }

    @Override
    public void robotInit() {
        // This is called once when the robot code initializes
        printBanner("eNqlVsGO3CAMvfMV3DIrZPtu9VNGwj1sL1UVaXuoVsrH1zYkgYTMrGbJikDAj4f97NkQQrxuwiwM/TcGYJBwbVEMkLCxIbE14EtLao/RIwARtW9AzHpsm+LzJhRWmP42+NyWpUOCFWnWu+5LSM+RGqZIzPN6H2B3XOFCXwBSc6iW5ri0e7V4zD5Dki0o6krYuR7munE9HBoXz/HoMeJqXhhKgupb3oArpiTHEY0h09HFtBEBN9x4xqPWXAmAQV0NKEPdNBHuPYw81IESUm5yZrUSr8dgGeA+HwcCiIvzbTBUT3E2d3M+3bRGlXBV198vKPOhaqu6dhm83DA5K6GetAYXrbM8jfzZHIOfGglA15iu9gS0GChamqEEVx/ybBINZnGhvnlnDYlqnNm2wOzCcUNvs2lW5JjTCVo4zSqpaRdbOKZjBZFThSqJwKmFs3jXBGjgZuxqTex1vlGrCPqW+t4Q6omnQre6XrzmDAFU0+sxCcNAh02NHgHskpR5YH8Qkrjvk4nCWi+XYZHfy8TDQp0uqjxe5Fe348HPUkuxZ6yOg/DwR5CrhC2OFQjta3hyl+IavzbHUsq+k4YwTd9BwEkhVIBvL0OQYWhPjxDycJx9fMsULwB0g/3luMQl2951XFZu+U1nvjC0X+KUFTzHu43iPdZxzmW+GBZFujS3pxyyeL+Ol7g+Mf4wL8bpbK274l0J05LRaPjYerfP/tyzkfJEzxrKcxR+ffz88/5v/vj9NIyTQUymh/BqIPUfBZgUJLxq7xRes765A0II/wGVOKOq");
        printBanner("eJzjUqAR0OPCL1+DTxRDEiIQEw8CmhCKOPPjE4Cs+HiFGBgbyMEiAKT0gUhXtwxkNFAowYYo8zXArBowCWbHgI2uAUOEJBDYIOnV1MdvONh/IKN04mviQS6EsYFAH+RaZEmwbAxCszoXEAAAFJdXwA==");
    }

    @Override
    public void autonomousInit() {
        // This is called once when the robot first enters autonomous mode
        loopInit("Autonomous", autonomousMode);
        
    }

    @Override
    public void autonomousPeriodic() {
        // This is called periodically while the robot is in autonomous mode
        loopExecute("Autonomous", autonomousMode);
    }

    @Override
    public void teleopInit() {
        // This is called once when the robot first enters teleoperated mode
        loopInit("Teleop", teleopMode);
    }

    @Override
    public void teleopPeriodic() {
        // This is called periodically while the robot is in teleopreated mode
        loopExecute("Teleop", teleopMode);
    }

    @Override
    public void testInit() {
        // This is called once when the robot enters test mode
        loopInit("Test", testMode);
    }

    @Override
    public void testPeriodic() {
        // This is called periodically while the robot is in test mode
        loopExecute("Test", testMode);
    }

    @Override
    public void disabledInit() {
        // This is called periodically while the robot is disabled
        System.out.println("DISABLED_INIT");
        loopEnd(currentModeName, currentMode);
        currentMode = null;
        currentModeName = null;
        loopInit("Disabled", null);
    }

    private void printBanner (String banner) {
        byte[] compRaw=Base64.getDecoder().decode(banner);
        InputStream compInflate=new InflaterInputStream(new ByteArrayInputStream(compRaw));
        ByteArrayOutputStream finalStream=new ByteArrayOutputStream();
        int copyInt=0;
        while(copyInt!=-1) {
        	try {
				copyInt=compInflate.read();
			} catch (IOException e) {
				e.printStackTrace();
				copyInt=-1;
			}
        	if (copyInt==-1) {
        		break;
        	}
        	finalStream.write(copyInt);
        }
        System.out.println(new String(finalStream.toByteArray()));
    }
}
