package com._604robotics.robotnik.prefabs.devices.laserrangefinder;

import edu.wpi.first.wpilibj.I2C;

/**
 * Used to interface with laser rangefinders based on the VL53L0X.
 * @see <a href='https://github.com/OlivierLD/raspberry-coffee/blob/master/I2C.SPI/src/i2c/sensor/VL53L0X.java'>https://github.com/OlivierLD/raspberry-coffee/blob/master/I2C.SPI/src/i2c/sensor/VL53L0X.java</a>
 * @see <a href='https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp'>https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp</a>
 */
public class LaserRFI2C {
    private Double distance;
    private I2C ranger; // Get it? Range Finder -> Ranger? :P Also, the default address is 0x29 on the sensor
                        // See: https://learn.adafruit.com/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout/arduino-code#connecting-multiple-sensors-4-15
    private Profile currentProfile;

    /* Constants for addresses */
    // <editor-fold description='Register Names'>
    public final static int VL53L0X_I2CADDR = 0x29;

    private final static int SYSRANGE_START = 0x00;
    private final static int SYSTEM_THRESH_HIGH = 0x0C;
    private final static int SYSTEM_THRESH_LOW = 0x0E;
    private final static int SYSTEM_SEQUENCE_CONFIG = 0x01;
    private final static int SYSTEM_RANGE_CONFIG = 0x09;
    private final static int SYSTEM_INTERMEASUREMENT_PERIOD = 0x04;
    private final static int SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A;
    private final static int GPIO_HV_MUX_ACTIVE_HIGH = 0x84;
    private final static int SYSTEM_INTERRUPT_CLEAR = 0x0B;
    private final static int RESULT_INTERRUPT_STATUS = 0x13;
    private final static int RESULT_RANGE_STATUS = 0x14;
    private final static int RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC;
    private final static int RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0;
    private final static int RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0;
    private final static int RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4;
    private final static int RESULT_PEAK_SIGNAL_RATE_REF = 0xB6;
    private final static int ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28;
    private final static int I2C_SLAVE_DEVICE_ADDRESS = 0x8A;
    private final static int MSRC_CONFIG_CONTROL = 0x60;
    private final static int PRE_RANGE_CONFIG_MIN_SNR = 0x27;
    private final static int PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56;
    private final static int PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57;
    private final static int PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64;
    private final static int FINAL_RANGE_CONFIG_MIN_SNR = 0x67;
    private final static int FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47;
    private final static int FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48;
    private final static int FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44;
    private final static int PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61;
    private final static int PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62;
    private final static int PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50;
    private final static int PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51;
    private final static int PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52;
    private final static int SYSTEM_HISTOGRAM_BIN = 0x81;
    private final static int HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33;
    private final static int HISTOGRAM_CONFIG_READOUT_CTRL = 0x55;
    private final static int FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70;
    private final static int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71;
    private final static int FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72;
    private final static int CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20;
    private final static int MSRC_CONFIG_TIMEOUT_MACROP = 0x46;
    private final static int SOFT_RESET_GO2_SOFT_RESET_N = 0xBF;
    private final static int IDENTIFICATION_MODEL_ID = 0xC0;
    private final static int IDENTIFICATION_REVISION_ID = 0xC2;
    private final static int OSC_CALIBRATE_VAL = 0xF8;
    private final static int GLOBAL_CONFIG_VCSEL_WIDTH = 0x32;
    private final static int GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0;
    private final static int GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1;
    private final static int GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2;
    private final static int GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3;
    private final static int GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4;
    private final static int GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5;
    private final static int GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6;
    private final static int DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E;
    private final static int DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F;
    private final static int POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80;
    private final static int VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89;
    private final static int ALGO_PHASECAL_LIM = 0x30;
    private final static int ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30;
    private final static int VCSEL_PERIOD_PRE_RANGE = 0;
    private final static int VCSEL_PERIOD_FINAL_RANGE = 1;
    // </editor-fold>

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

    public LaserRFI2C( int address, Profile profile ) {
        this.currentProfile = profile;
        this.ranger = new I2C(I2C.Port.kOnboard, address); // Pretty sure kOnboard...

        /* Check for expected values */
        /*boolean error = false;
        byte[] bytes = byte[1];
        ranger.read(0xC0, 1, bytes);
        if(bytes[0] != 0xEE ) error = true;
        ranger.read(0xC1, 1, bytes);
        if(bytes[0] != 0xAA ) error = true;
        ranger.read(0xC2, 1, bytes);
        if(bytes[0] != 0x10 ) error = true;
        if( error ) {
            throw new RuntimeException("Wiring is faulty: Expected register values not found.");
        }*/

        // Init I2C Standard Mode
        ranger.write(0x88, 0x00);
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
