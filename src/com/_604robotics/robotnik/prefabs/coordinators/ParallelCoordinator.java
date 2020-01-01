package com._604robotics.robotnik.prefabs.coordinators;

import java.util.HashMap;
import java.util.Map;

import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;

public class ParallelCoordinator extends Coordinator {
    private final Logger logger;

    private HashMap<Coordinator, Boolean> coordinators = new HashMap<Coordinator, Boolean>();

    public ParallelCoordinator(String name) {
        logger = new Logger(StatefulCoordinator.class, name);
      }
    
    public ParallelCoordinator(Class<?> klass) {
        this(klass.getSimpleName());
    }

    public ParallelCoordinator(String name, Coordinator... coordinators) {
        logger = new Logger(StatefulCoordinator.class, name);
      }
    
    public ParallelCoordinator(Class<?> klass, Coordinator... coordinators) {
        this(klass.getSimpleName());
    }

    public void addCoordinators(Coordinator... coordinators) {
        for ( Coordinator c : coordinators ) {
            this.coordinators.put(c, false);
        }
    }

    @Override
    public void begin() {
        logger.info("Begin");
        for (Map.Entry<Coordinator, Boolean> c : coordinators.entrySet()) {
            logger.info("Starting " + c.toString());
            c.getKey().start();
            c.setValue(true);
        }
    }

    @Override
    public boolean run() {
        for (Map.Entry<Coordinator, Boolean> c : coordinators.entrySet()) {
            c.setValue(c.getKey().execute());
        }

        return !coordinators.containsValue(true);
    }

    @Override
    public void end() {
        for (Map.Entry<Coordinator, Boolean> c : coordinators.entrySet()) {
            c.getKey().stop();
            logger.info("Stopped " + c.getKey().toString());
        }
        logger.info("End");
    }

}
