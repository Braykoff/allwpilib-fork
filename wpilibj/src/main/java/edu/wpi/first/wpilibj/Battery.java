// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.MathUtil;

/** Class for estimating and monitoring battery metrics throughout a match. */
public class Battery implements AutoCloseable {
  private final PowerDistribution m_distribution;
  private double m_resistance;

  private Notifier m_notifier;
  private double m_initialSoC;
  private double m_coulombs;
  private double m_prevTimestamp;
  private double m_prevCurrent;

  /**
   * Constructs a new Battery from the given power distribution component and internal resistance.
   * This will estimate the initial state of charge (SoC), so it is best to construct this while the
   * robot is disabled.
   *
   * @param distribution The power distribution component.
   * @param resistance The resistance of this battery, in ohms. If the length of the wire connecting
   *     the battery to the power distribution component is long enough to introduce another source
   *     of resistance, add that here. This can be calculated with the {@link
   *     #wireResistance(double, double)} method.
   */
  public Battery(PowerDistribution distribution, double resistance) {
    if (resistance < 0.0) {
      throw new IllegalArgumentException("Resistance must be >= 0.0");
    }

    m_distribution = requireNonNullParam(distribution, "distribution", "Battery");
    m_resistance = resistance;

    m_initialSoC = getStateOfCharge();
    m_prevCurrent = m_distribution.getTotalCurrent();
    m_prevTimestamp = Timer.getFPGATimestamp();

    m_notifier = new Notifier(this::updateCoulombCounter);
    m_notifier.startPeriodic(0.025);
  }

  /**
   * Constructs a new Battery from the given power distribution component. This will estimate the
   * initial state of charge (SoC), so it is best to construct this while the robot is disabled.
   *
   * <p>This will assume a default internal resistance of 0.015 Ohms. For better accuracy, measure
   * your battery's resistance with a multimeter or CTRE Battery Beak and use that instead.
   *
   * @param distribution The power distribution component.
   */
  public Battery(PowerDistribution distribution) {
    this(distribution, 0.015);
  }

  /**
   * Estimates the battery's state of charge. This is most accurate while the current draw is low,
   * such as when the robot is disabled.
   *
   * @return The estimated state of charge, as a percentage.
   */
  public double getStateOfCharge() {
    if (m_distribution.getTotalCurrent() > 1.0 && !DriverStation.isDisabled()) {
      DriverStation.reportWarning(
          "Estimating the battery's state of charge while under load can be inaccurate.\n"
              + "Consider disabling your robot first.",
          false);
    }

    // Get voltage, compensated for current draw
    double voltage =
        m_distribution.getVoltage() + (m_distribution.getTotalCurrent() * m_resistance);

    // TODO OCV to SoC lookup table
    double soc = (voltage * 0) + 0.5;

    return MathUtil.clamp(soc, 0.0, 1.3);
  }

  /**
   * Returns the total number of coulombs drawn from the battery. This is determined by integrating
   * the total current draw over time.
   *
   * @return The total number of coulombs drawn.
   */
  public double getCoulombs() {
    return m_coulombs;
  }

  /**
   * Estimates the amp-hour (Ah) capacity of the battery using the initial SoC, current SoC, and the
   * total number of coulombs drawn. This estimate is most accurate if called while the robot's
   * current draw is low, such as when the robot is disabled. An ideal FRC battery has an 18 Ah
   * capacity.
   *
   * @return The estimated Ah capacity
   */
  public double getAmpHours() {
    double soc = getStateOfCharge();
    double coulombs = getCoulombs();

    // Use SoC to extrapolate coulombs
    coulombs /= m_initialSoC - soc;

    // 3600 coulombs = 1 Ah
    return coulombs / 3600.0;
  }

  @Override
  public void close() {
    m_notifier.close();
  }

  /**
   * Method that runs periodically in another thread that counts the total number of coulombs drawn
   * from the battery by integrating current.
   */
  private void updateCoulombCounter() {
    double deltaT = Timer.getFPGATimestamp() - m_prevTimestamp;
    double current = m_distribution.getTotalCurrent();

    // Trapezoidal integration of current
    m_coulombs += 0.5 * (m_prevCurrent + current) * deltaT;
    m_prevTimestamp = Timer.getFPGATimestamp();
    m_prevCurrent = current;
  }

  /**
   * Estimates the resistance of a copper electrical wire from the specified length and
   * cross-sectional area.
   *
   * <pre>
   * R = ρ * (L/A)
   * </pre>
   *
   * @param length The length of the wire (meters).
   * @param area The cross-sectional area (meters^2)
   * @return The resistance of the wire (ohms).
   */
  public static double wireResistance(double length, double area) {
    return 1.72e-8 * (length / area);
  }
}
