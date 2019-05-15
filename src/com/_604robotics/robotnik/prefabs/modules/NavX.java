package com._604robotics.robotnik.prefabs.modules;

import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robot2019.modules.Dashboard;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;

import com.kauailabs.navx.frc.AHRS;


public class NavX extends Module {
    /* 
    This is a wrapper of the NavX IMU on the robot that provides some additional functionality over the default methods.
   Useful Resource: https://pdocs.kauailabs.com/navx-mxp/examples/data-monitor/

    */

    public AHRS ahrs;

    public NavX() {
        super(NavX.class);

        try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
            ahrs = new AHRS(SPI.Port.kMXP); 
            //ahrs = new AHRS(I2C.Port.kMXP);
            //ahrs = new AHRS(SerialPort.Port.kUSB1);
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
        

    }

    public float CompassHeadingCheck() {
        if ( ahrs.isMagnetometerCalibrated() ) {
            if ( ahrs.isMagneticDisturbance() ) {
                DriverStation.reportError("Too much magnetic disturbance for an accurate result.", true);
                return (ahrs.getYaw() + 180); // Should this output a value or just resturn null or 0?
            
            } else {
                return (ahrs.getCompassHeading());

            }

        } else {
            DriverStation.reportError("Please calibrate the Magnetometer before using the compass.", true);
            return (ahrs.getYaw() + 180);

        }
    }

    public float FusedHeadingCheck() {
        if ( ahrs.isMagnetometerCalibrated() ) {
            if ( ahrs.isMagneticDisturbance() ) {
                DriverStation.reportError("Too much magnetic disturbance for an accurate result.", true);
                return (ahrs.getYaw() + 180);
            
            } else {
                return (ahrs.getFusedHeading());

            }

        } else {
            DriverStation.reportError("Please calibrate the Magnetometer before using the compass with FUSED HEADING.", true);
            return (ahrs.getYaw() + 180);

        }
    }



}
