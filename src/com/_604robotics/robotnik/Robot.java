package com._604robotics.robotnik;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Base64;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.zip.InflaterInputStream;

import com._604robotics.robotnik.utils.Pair;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public abstract class Robot extends RobotBase {
    public static double DEFAULT_REPORT_INTERVAL = 5;

    private static final Logger logger = new Logger(Robot.class);
    
    private final NetworkTableInstance network = NetworkTableInstance.getDefault();
    private final NetworkTable table = network.getTable("robotnik");

    private final List<Module> modules = new ArrayList<>();
    private final IterationTimer iterationTimer;

    private final List<Pair<String, Coordinator>> systems = new ArrayList<>();

    private Coordinator autonomousMode;
    private Coordinator teleopMode;
    private Coordinator testMode;

    public Robot () {
        this(DEFAULT_REPORT_INTERVAL);
    }

    public Robot (double reportInterval) {
        iterationTimer = new IterationTimer(reportInterval);
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

    public void robotInit () {
        printBanner("eNqlVsGO3CAMvfMV3DIrZPtu9VNGwj1sL1UVaXuoVsrH1zYkgYTMrGbJikDAj4f97NkQQrxuwiwM/TcGYJBwbVEMkLCxIbE14EtLao/RIwARtW9AzHpsm+LzJhRWmP42+NyWpUOCFWnWu+5LSM+RGqZIzPN6H2B3XOFCXwBSc6iW5ri0e7V4zD5Dki0o6krYuR7munE9HBoXz/HoMeJqXhhKgupb3oArpiTHEY0h09HFtBEBN9x4xqPWXAmAQV0NKEPdNBHuPYw81IESUm5yZrUSr8dgGeA+HwcCiIvzbTBUT3E2d3M+3bRGlXBV198vKPOhaqu6dhm83DA5K6GetAYXrbM8jfzZHIOfGglA15iu9gS0GChamqEEVx/ybBINZnGhvnlnDYlqnNm2wOzCcUNvs2lW5JjTCVo4zSqpaRdbOKZjBZFThSqJwKmFs3jXBGjgZuxqTex1vlGrCPqW+t4Q6omnQre6XrzmDAFU0+sxCcNAh02NHgHskpR5YH8Qkrjvk4nCWi+XYZHfy8TDQp0uqjxe5Fe348HPUkuxZ6yOg/DwR5CrhC2OFQjta3hyl+IavzbHUsq+k4YwTd9BwEkhVIBvL0OQYWhPjxDycJx9fMsULwB0g/3luMQl2951XFZu+U1nvjC0X+KUFTzHu43iPdZxzmW+GBZFujS3pxyyeL+Ol7g+Mf4wL8bpbK274l0J05LRaPjYerfP/tyzkfJEzxrKcxR+ffz88/5v/vj9NIyTQUymh/BqIPUfBZgUJLxq7xRes765A0II/wGVOKOq");
        printBanner("eJzjUqAR0OPCL1+DTxRDEiIQEw8CmhCKOPPjE4Cs+HiFGBgbyMEiAKT0gUhXtwxkNFAowYYo8zXArBowCWbHgI2uAUOEJBDYIOnV1MdvONh/IKN04mviQS6EsYFAH+RaZEmwbAxCszoXEAAAFJdXwA==");
    }

    public void autonomous () {
        loop("Autonomous", autonomousMode, () -> isEnabled() && isAutonomous());
    }

    public void operatorControl () {
        loop("Teleop", teleopMode, () -> isEnabled() && isOperatorControl());
    }

    public void test () {
        loop("Test", testMode, () -> isEnabled() && isTest());
    }

    protected void disabled () {
        loop("Disabled", null, this::isDisabled);
    }

    private void loop (String name, Coordinator mode, Supplier<Boolean> active) {
        logger.info(name + " mode begin");

        for (Module module : modules) {
            Reliability.swallowThrowables(module::begin, "Error in begin() of module " + module.getName());
        }

        if (mode != null) {
            Reliability.swallowThrowables(mode::start, "Error starting " + name + " mode");
            iterationTimer.start();
        }

        for (Pair<String, Coordinator> system : systems) {
            Reliability.swallowThrowables(system.getValue()::start, "Error starting system " + system.getKey());
        }

        while (active.get()) {
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

            if (mode != null) {
                iterationTimer.sample(
                        loopTime -> logger.info("Loop time: " + loopTime * 1000 + " ms"));
            }
        }

        if (mode != null) {
            iterationTimer.stop();
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

        logger.info(name + " mode end");
    }
    
    @Override
    public void startCompetition() {
        robotInit();

        // Tell the DS that the robot is ready to be enabled
        HAL.observeUserProgramStarting();

        while (true) {
            if (isDisabled()) {
                m_ds.InDisabled(true);
                disabled();
                m_ds.InDisabled(false);

                while (isDisabled()) {
                    Timer.delay(0.01);
                }
            } else if (isAutonomous()) {
                m_ds.InAutonomous(true);
                autonomous();
                m_ds.InAutonomous(false);
                while (isAutonomous() && !isDisabled()) {
                    Timer.delay(0.01);
                }
            } else if (isTest()) {
                LiveWindow.setEnabled(true);
                Shuffleboard.enableActuatorWidgets();
                m_ds.InTest(true);
                test();
                m_ds.InTest(false);
                while (isTest() && isEnabled()) {
                    Timer.delay(0.01);
                }
                LiveWindow.setEnabled(false);
                Shuffleboard.disableActuatorWidgets();
            } else {
                m_ds.InOperatorControl(true);
                operatorControl();
                m_ds.InOperatorControl(false);
                while (isOperatorControl() && !isDisabled()) {
                    Timer.delay(0.01);
                }
            }
        } /* while loop */
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