package com._604robotics.robotnik.prefabs.devices;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class TalonPWMEncoder implements Encoder {
  // Use TalonSRX because WPI_TalonSRX extends this
  private final TalonSRX talon;

  public enum EncoderType {
    ABSOLUTE,
    RELATIVE
  }

  private final EncoderType encoderType;
  private boolean inverted;
  private double offset = 0;

  public TalonPWMEncoder(TalonSRX talon) {
    this(talon, EncoderType.RELATIVE);
  }

  public TalonPWMEncoder(TalonSRX talon, EncoderType type) {
    this.talon = talon;
    this.encoderType = type;
    this.inverted = false;
  }

  public boolean isInverted() {
    return inverted;
  }

  public void setInverted(boolean inverted) {
    this.inverted = inverted;
  }

  @Override
  public double getValue() {
    return getPosition();
  }

  // Use quadrature output for relative and pulse width output for absolute
  public double getPosition() {
    int multfactor = inverted ? -1 : 1;
    if (encoderType == EncoderType.ABSOLUTE) {
      return multfactor * (talon.getSensorCollection().getPulseWidthPosition() - offset);
    } else {
      return multfactor * talon.getSensorCollection().getQuadraturePosition();
    }
  }

  // Use quadrature output for relative and pulse width output for absolute
  public double getVelocity() {
    int multfactor = inverted ? -1 : 1;
    if (encoderType == EncoderType.ABSOLUTE) {
      return multfactor * talon.getSensorCollection().getPulseWidthVelocity();
    } else {
      return multfactor * talon.getSensorCollection().getQuadratureVelocity();
    }
  }

  public void zero() {
    if (encoderType == EncoderType.ABSOLUTE) {
      offset = (talon.getSensorCollection().getPulseWidthPosition());
    }
  }

  public void zero(double value) {
    if (encoderType == EncoderType.ABSOLUTE) {
      int multfactor = inverted ? -1 : 1;
      offset = talon.getSensorCollection().getPulseWidthPosition() - (multfactor * value);
    }
  }

  public double getOffset() {
    if (encoderType == EncoderType.ABSOLUTE) {
      return offset;
    } else {
      return 0;
    }
  }
}
