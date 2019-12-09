package com._604robotics.robot2019.modules;

import com._604robotics.robotnik.Module;
import edu.wpi.first.cameraserver.CameraServer;

public class Camera extends Module {

  public Camera() {
    super(Camera.class);
    CameraServer.getInstance().startAutomaticCapture("video0", "/dev/video0");
  }

  public Camera(String devName) {
    super(Camera.class);
    CameraServer.getInstance().startAutomaticCapture(devName, "/dev/" + devName);
  }
}
