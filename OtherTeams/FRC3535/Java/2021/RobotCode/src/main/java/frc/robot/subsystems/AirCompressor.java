package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;

/**
 * By default, the compressor automatically turns itself on/off depending on the
 * pressure, but it can be disabled in a power-intensive situation
 */
public class AirCompressor extends Subsystem {

  public static Compressor compressor = new Compressor(Constants.pcmPort);

  /**
   * By default, the compressor automatically turns itself on/off depending on the
   * pressure, but it can be disabled in a power-intensive situation
   */
  public AirCompressor() {}

  @Override
  public void initDefaultCommand() {}

  /**
   * Enables the compressor to turn on when the air pressure gets below a given
   * pressure (Default 90 psi)
   */
  public static void enable() {
    compressor.start();
  }

  public static void clearFaults() {
    compressor.clearAllPCMStickyFaults();
  }

  /**
   * Disables the compressor
   */
  public static void disable() {
    compressor.stop();
  }

  /**
   * Gets the status of the compressor
   */
  public static boolean status() {
    return compressor.enabled();
  }
}
