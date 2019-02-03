package com._604robotics.robotnik.prefabs.devices;

import com._604robotics.robotnik.utils.Pair;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

import java.util.*;

/**
 * <p>Designed to avoid the issue of one encoder failing, by using
 * a second encoder at the same time. If one value jumps by a ton,
 * but the other one does NOT, we assume the first encoder is broken
 * and DO NOT use that value any more.</p>
 *
 * <p>Only use this class if both encoders are measuring the SAME
 * value. Otherwise, this class is useless and will only hurt you.</p>
 */
public class RedundantEncoder implements PIDSource, Encoder {
    // NOTE Assumes encoders where 250 clicks is one rotation
    private static final double LARGE_CLICK_DIFF = 250; // One rotation difference

    // Maybe just use a boolean?
    private Pair<Boolean, Double> maxClicks;
    private Pair<Boolean, Double> minClicks;

    private Map<Encoder, Double> prevEncoderVal;
    private Map<Encoder, Double> currentEncoderVal;
    private Map<Encoder, Boolean> encoderStatus;

    private PIDSourceType sourceType;

    public RedundantEncoder(Encoder... encoders) {
        if( encoders.length > 2 ) {
            throw new RuntimeException("RedundantEncoder does not currently support more than 2 encoders");
        } else if(encoders.length < 1 ) {
            throw new RuntimeException("RedundantEncoder needs at least one encoder (though you should have at least 2)");
        } else if( !samePIDType(encoders) ) {
            throw new RuntimeException("All encoders in RedundantEncoder must have the same PID type");
        } else if( encoders[0].getPIDSourceType().equals(PIDSourceType.kRate) ) {
            throw new RuntimeException("RedundantEncoder does not currently support kRate based encoders");
        }

        sourceType = encoders[0].getPIDSourceType();

        prevEncoderVal = new HashMap<>(encoders.length);
        currentEncoderVal = new HashMap<>(encoders.length);
        encoderStatus = new HashMap<>(encoders.length);

        maxClicks = new Pair<>(false, 0.0);
        minClicks = new Pair<>(false, 0.0);

        for( Encoder e : encoders ) {
            prevEncoderVal.put(e, e.getValue());
            encoderStatus.put(e, true);
        }
    }

    public void setMaximum(double clicks) {
        maxClicks = new Pair<>(true, clicks);
    }

    public void setMinimum(double clicks) {
        minClicks = new Pair<>(true, clicks);
    }

    /**
     * Checks two encoders, and if one has failed, sets the {@code encoderStatus} value
     * to false
     */
    private void checkEncoders() {
        // Update encoder values
        prevEncoderVal.putAll(currentEncoderVal);
        for( Encoder e : currentEncoderVal.keySet() ) {
            currentEncoderVal.put(e, e.getValue());
        }

        List<Encoder> trusted = getCurrentlyTrusted(encoderStatus);
        Map<Encoder, Double> diffValues = new HashMap<>(trusted.size());

        for( Encoder e : trusted ) {
            // Only care about change from zero, so absolute values is best
            diffValues.put(e, Math.abs(currentEncoderVal.get(e) - prevEncoderVal.get(e)));

            // If values are over the guaranteed values, we can immediately assume the encoder is bad
            if( maxClicks.getKey() && currentEncoderVal.get(e) > maxClicks.getValue() ) {
                encoderStatus.put(e, false);
            } else if( minClicks.getKey() && currentEncoderVal.get(e) < minClicks.getValue() ) {
                encoderStatus.put(e, false);
            }
        }

        // Assumes only two encoders are present
        if( trusted.size() > 1 && (diffValues.get(trusted.get(0)) - diffValues.get(trusted.get(1))) > LARGE_CLICK_DIFF ) {
            // Toss the value that changed the most, if the difference is too big
            if( diffValues.get(trusted.get(0)) > diffValues.get(trusted.get(1)) ) {
                encoderStatus.put(trusted.get(0), false);
            } else {
                encoderStatus.put(trusted.get(1), false);
            }
        }

    }

    private List<Encoder> getCurrentlyTrusted( Map<Encoder, Boolean> encoders) {
        List<Encoder> result = new ArrayList<>();

        for( Encoder e : encoders.keySet() ) {
            if( encoders.get(e) ) {
                result.add(e);
            }
        }

        return result;
    }

    private boolean samePIDType(Encoder[] encoders) {
        PIDSourceType first = encoders[0].getPIDSourceType();

        for( int i = 1; i < encoders.length; ++i ) {
            if( !first.equals(encoders[i].getPIDSourceType()) ) {
                return false;
            }
        }

        return true;
    }

    public List<Encoder> getEncoders() {
        return new ArrayList<>(encoderStatus.keySet());
    }

    @Override
    public double getValue() {
        List<Encoder> trusted = getCurrentlyTrusted(encoderStatus);

        if( trusted.size() > 1 ) {
            checkEncoders();
            trusted = getCurrentlyTrusted(encoderStatus);
        }

        double total = 0;

        for( Encoder e : trusted ) {
            total += currentEncoderVal.get(e);
        }

        // Get the average value of all trusted encoders
        return total / trusted.size();
    }

    @Override
    public void setPIDSourceType( PIDSourceType pidSource ) {
        // TODO Implement rate based encoders
        if (sourceType != PIDSourceType.kDisplacement) {
            throw new IllegalArgumentException("RedundantEncoder class only implements PIDSourceType.kDisplacement");
        }
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return sourceType;
    }

    @Override
    public double pidGet() {
        return getValue();
    }
}
