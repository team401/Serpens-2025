package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

/** Geometry utils that need to be added to CopperCore, but haven't been yet */
public class GeomUtil {
  /**
   * Take the 2D Cross Product of two Translations
   *
   * <p>This is defined as: a x b = ax*by - ay*bx
   *
   * @param a The first translation (a), representing a 2d vector
   * @param b The second translation (b), representing a 2d vector
   * @return If the vectors are collinear, 0. If b is clockwise from a, a negative value. If b is
   *     counterclockwise from a, a positive value.
   */
  public static double cross(Translation2d a, Translation2d b) {
    // TODO: move this method to coppercore geometry
    return a.getX() * b.getY() - a.getY() * b.getX();
  }
}
