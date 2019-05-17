package com._604robotics.robotnik;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com._604robotics.robotnik.prefabs.flow.SmartTimer;

public class NavX extends AHRS{

    private SmartTimer compassTime;
    private SmartTimer fusedTime;  

    private float prevReadoutCompass = 0; // Needs to be 720 to switch to yaw readout if now previous value has been stored
    private float prevReadoutFused = 0; // Needs to be 720 to switch to yaw readout if now previous value has been stored

    private double switchtime = 2;


    public NavX(SPI.Port port) { //Change to I2C.Port for I2C, SPI.Port for SPI, and SerialPort.port for Serial/USB
        super(port);

        compassTime = new SmartTimer();
        fusedTime = new SmartTimer();
    }
    
    public float getCompassHeading() {

        if ( this.isMagnetometerCalibrated() ) {
            if ( this.isMagneticDisturbance() ) {
                DriverStation.reportError("Too much magnetic disturbance for an accurate result.", true);

                 /* This statment is neede to check if the previous value should be used based on the tim pa */
                compassTime.startIfNotRunning();

                if ( compassTime.hasReachedTime(switchtime) )  { 
                    return(this.getYaw() + 180);
                } else {
                    return (prevReadoutCompass);
                }

            } else {
                compassTime.stopAndReset();
                prevReadoutCompass = this.getCompassHeading();
                return (this.getCompassHeading());

            }

        } else {
            DriverStation.reportError("Please calibrate the Magnetometer before using the compass.", true);

            compassTime.startIfNotRunning();
                if ( compassTime.hasReachedTime(switchtime) )  { 
                    return(this.getYaw() + 180);
                } else {
                    return (prevReadoutCompass);
                }

        }
    }

    public float getFusedHeading() {

        if ( this.isMagnetometerCalibrated() ) {
            if ( this.isMagneticDisturbance() ) {
                DriverStation.reportError("Too much magnetic disturbance for an accurate result(fh).", true);
                
                fusedTime.startIfNotRunning();

                if ( fusedTime.hasReachedTime(switchtime) )  { 
                    return(this.getYaw() + 180);
                } else {
                    return (prevReadoutFused);
                }

            } else {
                fusedTime.stopAndReset();
                prevReadoutFused = this.getFusedHeading();
                return (this.getFusedHeading());

            }

        } else {
            DriverStation.reportError("Please calibrate the Magnetometer before using the FUSED heading.", true);
            
            fusedTime.startIfNotRunning();

            if ( fusedTime.hasReachedTime(switchtime) )  { 
                return(this.getYaw() + 180);
            } else {
                return (prevReadoutFused);
            }
        }
    }
}