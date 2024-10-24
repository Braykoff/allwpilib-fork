// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.hal.LEDJNI;
import edu.wpi.first.hal.PowerJNI;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

/** Contains functions for roboRIO functionality. */
public final class RobotController {
  // Mutable measures
  private static final MutTime m_mutFPGATime = Microseconds.mutable(0.0);
  private static final MutVoltage m_mutBatteryVoltage = Volts.mutable(0.0);
  private static final MutVoltage m_mutInputVoltage = Volts.mutable(0.0);
  private static final MutCurrent m_mutInputCurrent = Amps.mutable(0.0);
  private static final MutVoltage m_mutVoltage3V3 = Volts.mutable(0.0);
  private static final MutCurrent m_mutCurrent3V3 = Amps.mutable(0.0);
  private static final MutVoltage m_mutVoltage5V = Volts.mutable(0.0);
  private static final MutCurrent m_mutCurrent5V = Amps.mutable(0.0);
  private static final MutVoltage m_mutVoltage6V = Volts.mutable(0.0);
  private static final MutCurrent m_mutCurrent6V = Amps.mutable(0.0);
  private static final MutVoltage m_mutBrownoutVoltage = Volts.mutable(0.0);
  private static final MutTemperature m_mutCPUTemp = Celsius.mutable(0.0);

  private RobotController() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Return the FPGA Version number. For now, expect this to be the current year.
   *
   * @return FPGA Version number.
   */
  public static int getFPGAVersion() {
    return HALUtil.getFPGAVersion();
  }

  /**
   * Return the FPGA Revision number. The format of the revision is 3 numbers. The 12 most
   * significant bits are the Major Revision. the next 8 bits are the Minor Revision. The 12 least
   * significant bits are the Build Number.
   *
   * @return FPGA Revision number.
   */
  public static long getFPGARevision() {
    return HALUtil.getFPGARevision();
  }

  /**
   * Return the serial number of the roboRIO.
   *
   * @return The serial number of the roboRIO.
   */
  public static String getSerialNumber() {
    return HALUtil.getSerialNumber();
  }

  /**
   * Return the comments from the roboRIO web interface.
   *
   * <p>The comments string is cached after the first call to this function on the RoboRIO - restart
   * the robot code to reload the comments string after changing it in the web interface.
   *
   * @return the comments from the roboRIO web interface.
   */
  public static String getComments() {
    return HALUtil.getComments();
  }

  /**
   * Returns the team number configured for the robot controller.
   *
   * @return team number, or 0 if not found.
   */
  public static int getTeamNumber() {
    return HALUtil.getTeamNumber();
  }

  /**
   * Read the microsecond timer from the FPGA.
   *
   * @return The current time in microseconds according to the FPGA.
   */
  public static long getFPGATime() {
    return HALUtil.getFPGATime();
  }

  /**
   * Read the microsecond timer in a mutable measure from the FPGA.
   *
   * @return The current time according to the FPGA in a mutable measure.
   */
  public static MutTime getMeasureFPGATime() {
    m_mutFPGATime.mut_replace(HALUtil.getFPGATime(), Microseconds);
    return m_mutFPGATime;
  }

  /**
   * Get the state of the "USER" button on the roboRIO.
   *
   * <p>Warning: the User Button is used to stop user programs from automatically loading if it is
   * held for more then 5 seconds. Because of this, it's not recommended to be used by teams for any
   * other purpose.
   *
   * @return true if the button is currently pressed down
   */
  public static boolean getUserButton() {
    return HALUtil.getFPGAButton();
  }

  /**
   * Read the battery voltage.
   *
   * @return The battery voltage in Volts.
   */
  public static double getBatteryVoltage() {
    return PowerJNI.getVinVoltage();
  }

  /**
   * Read the battery voltage in a mutable measure.
   *
   * @return The battery voltage in a mutable measure.
   */
  public static MutVoltage getMeasureBatteryVoltage() {
    m_mutBatteryVoltage.mut_replace(PowerJNI.getVinVoltage(), Volts);
    return m_mutBatteryVoltage;
  }

  /**
   * Gets a value indicating whether the FPGA outputs are enabled. The outputs may be disabled if
   * the robot is disabled or e-stopped, the watchdog has expired, or if the roboRIO browns out.
   *
   * @return True if the FPGA outputs are enabled.
   */
  public static boolean isSysActive() {
    return HAL.getSystemActive();
  }

  /**
   * Check if the system is browned out.
   *
   * @return True if the system is browned out
   */
  public static boolean isBrownedOut() {
    return HAL.getBrownedOut();
  }

  /**
   * Gets the number of times the system has been disabled due to communication errors with the
   * Driver Station.
   *
   * @return number of disables due to communication errors.
   */
  public static int getCommsDisableCount() {
    return HAL.getCommsDisableCount();
  }

  /**
   * Gets the current state of the Robot Signal Light (RSL).
   *
   * @return The current state of the RSL- true if on, false if off
   */
  public static boolean getRSLState() {
    return HAL.getRSLState();
  }

  /**
   * Gets if the system time is valid.
   *
   * @return True if the system time is valid, false otherwise
   */
  public static boolean isSystemTimeValid() {
    return HAL.getSystemTimeValid();
  }

  /**
   * Get the input voltage to the robot controller.
   *
   * @return The controller input voltage value in Volts
   */
  public static double getInputVoltage() {
    return PowerJNI.getVinVoltage();
  }

  /**
   * Get the input voltage to the robot controller in a mutable measure.
   *
   * @return The controller input voltage value in a mutable measure.
   */
  public static MutVoltage getMeasureInputVoltage() {
    m_mutInputVoltage.mut_replace(PowerJNI.getVinVoltage(), Volts);
    return m_mutInputVoltage;
  }

  /**
   * Get the input current to the robot controller.
   *
   * @return The controller input current value in Amps
   */
  public static double getInputCurrent() {
    return PowerJNI.getVinCurrent();
  }

  /**
   * Get the input current to the robot controller in a mutable measure.
   *
   * @return The controller input current value in a mutable measure.
   */
  public static MutCurrent getMeasureInputCurrent() {
    m_mutInputCurrent.mut_replace(PowerJNI.getVinCurrent(), Amps);
    return m_mutInputCurrent;
  }

  /**
   * Get the voltage of the 3.3V rail.
   *
   * @return The controller 3.3V rail voltage value in Volts
   */
  public static double getVoltage3V3() {
    return PowerJNI.getUserVoltage3V3();
  }

  /**
   * Get the voltage in a mutable measure of the 3.3V rail.
   *
   * @return The controller 3.3V rail voltage value in a mutable measure.
   */
  public static MutVoltage getMeasureVoltage3V3() {
    m_mutVoltage3V3.mut_replace(PowerJNI.getUserVoltage3V3(), Volts);
    return m_mutVoltage3V3;
  }

  /**
   * Get the current output of the 3.3V rail.
   *
   * @return The controller 3.3V rail output current value in Amps
   */
  public static double getCurrent3V3() {
    return PowerJNI.getUserCurrent3V3();
  }

  /**
   * Get the current output in a mutable measure of the 3.3V rail.
   *
   * @return The controller 3.3V rail output current value in a mutable measure.
   */
  public static MutCurrent getMeasureCurrent3V3() {
    m_mutCurrent3V3.mut_replace(PowerJNI.getUserCurrent3V3(), Amps);
    return m_mutCurrent3V3;
  }

  /**
   * Enables or disables the 3.3V rail.
   *
   * @param enabled whether to enable the 3.3V rail.
   */
  public static void setEnabled3V3(boolean enabled) {
    PowerJNI.setUserEnabled3V3(enabled);
  }

  /**
   * Get the enabled state of the 3.3V rail. The rail may be disabled due to a controller brownout,
   * a short circuit on the rail, or controller over-voltage.
   *
   * @return The controller 3.3V rail enabled value
   */
  public static boolean getEnabled3V3() {
    return PowerJNI.getUserActive3V3();
  }

  /**
   * Get the count of the total current faults on the 3.3V rail since the code started.
   *
   * @return The number of faults
   */
  public static int getFaultCount3V3() {
    return PowerJNI.getUserCurrentFaults3V3();
  }

  /**
   * Get the voltage of the 5V rail.
   *
   * @return The controller 5V rail voltage value in Volts
   */
  public static double getVoltage5V() {
    return PowerJNI.getUserVoltage5V();
  }

  /**
   * Get the voltage in a mutable measure of the 5V rail.
   *
   * @return The controller 5V rail voltage value in a mutable measure.
   */
  public static MutVoltage getMeasureVoltage5V() {
    m_mutVoltage5V.mut_replace(PowerJNI.getUserVoltage5V(), Volts);
    return m_mutVoltage5V;
  }

  /**
   * Get the current output of the 5V rail.
   *
   * @return The controller 5V rail output current value in Amps
   */
  public static double getCurrent5V() {
    return PowerJNI.getUserCurrent5V();
  }

  /**
   * Get the current output in a mutable measure of the 5V rail.
   *
   * @return The controller 5V rail output current value in a mutable measure.
   */
  public static MutCurrent getMeasureCurrent5V() {
    m_mutCurrent5V.mut_replace(PowerJNI.getUserCurrent5V(), Amps);
    return m_mutCurrent5V;
  }

  /**
   * Enables or disables the 5V rail.
   *
   * @param enabled whether to enable the 5V rail.
   */
  public static void setEnabled5V(boolean enabled) {
    PowerJNI.setUserEnabled5V(enabled);
  }

  /**
   * Get the enabled state of the 5V rail. The rail may be disabled due to a controller brownout, a
   * short circuit on the rail, or controller over-voltage.
   *
   * @return The controller 5V rail enabled value
   */
  public static boolean getEnabled5V() {
    return PowerJNI.getUserActive5V();
  }

  /**
   * Get the count of the total current faults on the 5V rail since the code started.
   *
   * @return The number of faults
   */
  public static int getFaultCount5V() {
    return PowerJNI.getUserCurrentFaults5V();
  }

  /**
   * Get the voltage of the 6V rail.
   *
   * @return The controller 6V rail voltage value in Volts
   */
  public static double getVoltage6V() {
    return PowerJNI.getUserVoltage6V();
  }

  /**
   * Get the voltage in a mutable measure of the 6V rail.
   *
   * @return The controller 6V rail voltage value in a mutable measure.
   */
  public static MutVoltage getMeasureVoltage6V() {
    m_mutVoltage6V.mut_replace(PowerJNI.getUserVoltage6V(), Volts);
    return m_mutVoltage6V;
  }

  /**
   * Get the current output of the 6V rail.
   *
   * @return The controller 6V rail output current value in Amps
   */
  public static double getCurrent6V() {
    return PowerJNI.getUserCurrent6V();
  }

  /**
   * Get the current output in a mutable measure of the 6V rail.
   *
   * @return The controller 6V rail output current value in a mutable measure.
   */
  public static MutCurrent getMeasureCurrent6V() {
    m_mutCurrent6V.mut_replace(PowerJNI.getUserCurrent6V(), Amps);
    return m_mutCurrent6V;
  }

  /**
   * Enables or disables the 6V rail.
   *
   * @param enabled whether to enable the 6V rail.
   */
  public static void setEnabled6V(boolean enabled) {
    PowerJNI.setUserEnabled6V(enabled);
  }

  /**
   * Get the enabled state of the 6V rail. The rail may be disabled due to a controller brownout, a
   * short circuit on the rail, or controller over-voltage.
   *
   * @return The controller 6V rail enabled value
   */
  public static boolean getEnabled6V() {
    return PowerJNI.getUserActive6V();
  }

  /**
   * Get the count of the total current faults on the 6V rail since the code started.
   *
   * @return The number of faults
   */
  public static int getFaultCount6V() {
    return PowerJNI.getUserCurrentFaults6V();
  }

  /** Reset the overcurrent fault counters for all user rails to 0. */
  public static void resetRailFaultCounts() {
    PowerJNI.resetUserCurrentFaults();
  }

  /**
   * Get the current brownout voltage setting.
   *
   * @return The brownout voltage
   */
  public static double getBrownoutVoltage() {
    return PowerJNI.getBrownoutVoltage();
  }

  /**
   * Get the current brownout voltage setting in a mutable measure.
   *
   * @return The brownout voltage in a mutable measure.
   */
  public static MutVoltage getMeasureBrownoutVoltage() {
    m_mutBrownoutVoltage.mut_replace(PowerJNI.getBrownoutVoltage(), Volts);
    return m_mutBrownoutVoltage;
  }

  /**
   * Set the voltage the roboRIO will brownout and disable all outputs.
   *
   * <p>Note that this only does anything on the roboRIO 2. On the roboRIO it is a no-op.
   *
   * @param brownoutVoltage The brownout voltage
   */
  public static void setBrownoutVoltage(double brownoutVoltage) {
    PowerJNI.setBrownoutVoltage(brownoutVoltage);
  }

  /**
   * Set the voltage in a measure the roboRIO will brownout and disable all outputs.
   *
   * <p>Note that this only does anything on the roboRIO 2. On the roboRIO it is a no-op.
   *
   * @param brownoutVoltage The brownout voltage in a measure
   */
  public static void setBrownoutVoltage(Voltage brownoutVoltage) {
    PowerJNI.setBrownoutVoltage(brownoutVoltage.baseUnitMagnitude());
  }

  /**
   * Get the current CPU temperature in degrees Celsius.
   *
   * @return current CPU temperature in degrees Celsius
   */
  public static double getCPUTemp() {
    return PowerJNI.getCPUTemp();
  }

  /**
   * Get the current CPU temperature in a mutable measure.
   *
   * @return current CPU temperature in a mutable measure.
   */
  public static MutTemperature getMeasureCPUTemp() {
    m_mutCPUTemp.mut_replace(PowerJNI.getCPUTemp(), Celsius);
    return m_mutCPUTemp;
  }

  /** State for the radio led. */
  public enum RadioLEDState {
    /** Off. */
    kOff(LEDJNI.RADIO_LED_STATE_OFF),
    /** Green. */
    kGreen(LEDJNI.RADIO_LED_STATE_GREEN),
    /** Red. */
    kRed(LEDJNI.RADIO_LED_STATE_RED),
    /** Orange. */
    kOrange(LEDJNI.RADIO_LED_STATE_ORANGE);

    /** The native value for this state. */
    public final int value;

    RadioLEDState(int value) {
      this.value = value;
    }

    /**
     * Gets a state from an int value.
     *
     * @param value int value
     * @return state
     */
    public static RadioLEDState fromValue(int value) {
      return switch (value) {
        case LEDJNI.RADIO_LED_STATE_OFF -> RadioLEDState.kOff;
        case LEDJNI.RADIO_LED_STATE_GREEN -> RadioLEDState.kGreen;
        case LEDJNI.RADIO_LED_STATE_RED -> RadioLEDState.kRed;
        case LEDJNI.RADIO_LED_STATE_ORANGE -> RadioLEDState.kOrange;
        default -> RadioLEDState.kOff;
      };
    }
  }

  /**
   * Set the state of the "Radio" LED. On the RoboRIO, this writes to sysfs, so this function should
   * not be called multiple times per loop cycle to avoid overruns.
   *
   * @param state The state to set the LED to.
   */
  public static void setRadioLEDState(RadioLEDState state) {
    LEDJNI.setRadioLEDState(state.value);
  }

  /**
   * Get the state of the "Radio" LED. On the RoboRIO, this reads from sysfs, so this function
   * should not be called multiple times per loop cycle to avoid overruns.
   *
   * @return The state of the LED.
   */
  public static RadioLEDState getRadioLEDState() {
    return RadioLEDState.fromValue(LEDJNI.getRadioLEDState());
  }

  /**
   * Get the current status of the CAN bus.
   *
   * @return The status of the CAN bus
   */
  public static CANStatus getCANStatus() {
    CANStatus status = new CANStatus();
    CANJNI.getCANStatus(status);
    return status;
  }
}
