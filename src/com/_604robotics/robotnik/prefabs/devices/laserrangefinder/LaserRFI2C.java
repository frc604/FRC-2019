package com._604robotics.robotnik.prefabs.devices.laserrangefinder;

import edu.wpi.first.wpilibj.I2C;

public class LaserRFI2C {
    private Double distance;
    private I2C ranger; // Get it? Range Finder -> Ranger? :P Also, the default address is 0x29 on the sensor
                        // See: https://learn.adafruit.com/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout/arduino-code#connecting-multiple-sensors-4-15
    private Profile currentProfile;

    /**
     * The built in ranging profiles.
     * All values from the datasheet, and should be checked imperically.
     */
    public enum Profile {
        /**
         * 30ms, 1.2m, +/- 6%
         */
        DEFAULT,

        /**
         * 20ms, 1.2m, +/- 5%
         */
        FAST,

        /**
         * 200ms, 1.2m, +/- <3%
         */
        ACCURATE,

        /**
         * 33ms, 2m, +/- 6%
         */
        LONG,
    }

    private LaserRFI2C() {} // Must pass argument

    public LaserRFI2C( I2C ranger, Profile profile ) {
        this.ranger = ranger;
        this.currentProfile = profile;
    }

    /**
     * If continuous scan is enabled, gets the value of the last scan.
     * Otherwise, returns the previous scan value
     * @return The current value read by the rangefinder
     */
    public Double getDistance() {
        return null; // TODO
    }

    /**
     * If continuous scan is disabled, starts a scan.
     * Otherwise, does nothing
     */
    public void scan() {
        ranger.write(0,0);
    }

    /**
     * Begins a continuous scan, with no delay between consecutive scans.
     */
    public void scanContinuous() {

    }

    /**
     * Begins a continuous scan, with a delay between consecutive scans.
     * @param delay The time in milliseconds between scans
     */
    public void scanContinuous( Double delay ) {

    }

    /**
     * Sets the ranging profile of the rangefinder to the value of the enum
     * @param profile The profile to change to
     */
    public void rangingProfile( Profile profile ) {

    }

    /**
     * Stops scanning.
     * Mostly useful when using continuous mode.
     * The current scan will finish before stopping.
     */
    public void stopScan() {

    }

    /**
     * Checks to see if the rangefinder has finished a scan.
     * If scanning continuously, will return true after one scan is done.
     *
     * This uses the polling functionality of the rangefinder API.
     * It is possible to use an interrupt instead.
     * @return If the scan has a value ready to be fetched
     */
    public boolean scanFinished() {

    }

}
