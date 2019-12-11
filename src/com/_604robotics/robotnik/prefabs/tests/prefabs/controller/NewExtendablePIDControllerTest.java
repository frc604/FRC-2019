/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* This is 604's custom version to test the re-written multithreaded          */
/* controllers                                                                */
/*----------------------------------------------------------------------------*/


package edu.wpi.first.wpilibj.controller;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class PIDInputOutputTest {
  private PIDController m_controller;

  @BeforeEach
  void setUp() {
    m_controller = new PIDController(0, 0, 0);
  }

  @Test
  void continuousInputTest() {
    m_controller.setP(1);
    m_controller.enableContinuousInput(-180, 180);

    assertTrue(m_controller.calculate(-179, 179) < 0.0);
  }

  @Test
  void proportionalGainOutputTest() {
    m_controller.setP(4);

    assertEquals(-0.1, m_controller.calculate(0.025, 0), 1e-5);
  }

  @Test
  void integralGainOutputTest() {
    m_controller.setI(4);

    double out = 0;

    for (int i = 0; i < 5; i++) {
      out = m_controller.calculate(0.025, 0);
    }

    assertEquals(-0.5 * m_controller.getPeriod(), out, 1e-5);
  }

  @Test
  void derivativeGainOutputTest() {
    m_controller.setD(4);

    m_controller.calculate(0, 0);

    assertEquals(-0.01 / m_controller.getPeriod(), m_controller.calculate(0.0025, 0), 1e-5);
  }
}
