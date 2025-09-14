package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Utility class to unwrap DriverStation.getAlliance
 *
 * <p>This class defaults to assuming red if no alliance is present, because red is the higher seed
 * and WE PLAY TO WIN
 */
public class AllianceUtil {
  /**
   * Is the current alliance red?
   *
   * @return True if the alliance is red or there is no alliance, false if the alliance is blue
   */
  public static boolean isRed() {
    return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;
  }
}
