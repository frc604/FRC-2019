package com._604robotics.robotnik.prefabs.devices;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class TalonPWMEncoder implements Encoder {
  // Use m_talonSRX because WPI_m_talonSRX extends this
  private final TalonSRX m_talon;

  private double m_distancePerClick = 1;

  public enum EncoderType {
    ABSOLUTE,
    RELATIVE
  }

  private final EncoderType encoderType;
  private boolean inverted;
  private double offset = 0;

  public TalonPWMEncoder(TalonSRX m_talon) {
    this(m_talon, EncoderType.RELATIVE);
  }

  public TalonPWMEncoder(TalonSRX talon, EncoderType type) {
    this.m_talon = talon;
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

  public void setDistancePerClick(double distancePerClick) {
    m_distancePerClick = distancePerClick;
  }

  public double getDistancePerClick() {
    return m_distancePerClick;
  }

  // Use quadrature output for relative and pulse width output for absolute
  public double getPosition() {
    int multfactor = inverted ? -1 : 1;
    if (encoderType == EncoderType.ABSOLUTE) {
      return multfactor * (getPWMPos() - offset);
    } else {
      return multfactor * getQuadraturePos();
    }
  }

  // Use quadrature output for relative and pulse width output for absolute
  public double getVelocity() {
    int multfactor = inverted ? -1 : 1;
    if (encoderType == EncoderType.ABSOLUTE) {
      return multfactor * getPWMVel();
    } else {
      return multfactor * getQuadratureVel();
    }
  }

  public void zero() {
    if (encoderType == EncoderType.ABSOLUTE) {
      offset = (getPWMPos());
    }
  }

  public void zero(double value) {
    if (encoderType == EncoderType.ABSOLUTE) {
      int multfactor = inverted ? -1 : 1;
      offset = getPWMPos() - (multfactor * value);
    }
  }

  public double getOffset() {
    if (encoderType == EncoderType.ABSOLUTE) {
      return offset;
    } else {
      return 0;
    }
  }

  private double getPWMPos() {
    return m_talon.getSensorCollection().getPulseWidthPosition() * m_distancePerClick;
  }

  private double getQuadraturePos() {
    return m_talon.getSensorCollection().getQuadraturePosition() * m_distancePerClick;
  }

  private double getPWMVel() {
    // Velocity is over 100ms(0.1s) so multiply by 10 to get seconds.
    return m_talon.getSensorCollection().getPulseWidthVelocity() * 10 * m_distancePerClick;
  }

  private double getQuadratureVel() {
    // Velocity is over 100ms(0.1s) so multiply by 10 to get seconds.
    return m_talon.getSensorCollection().getQuadratureVelocity() * 10 * m_distancePerClick;
  }
}
