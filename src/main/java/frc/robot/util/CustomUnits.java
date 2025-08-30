package frc.robot.util;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.UnitBuilder;

public class CustomUnits {
  /** Rotations Per Minute (Rotations Per Second * (60 Seconds / 1 Minute)) */
  public static final AngularVelocityUnit RotationsPerMinute =
      new UnitBuilder<>(RotationsPerSecond)
          .named("RotationsPerMinute")
          .symbol("rpm")
          .fromBase(
              rotationsPerSecond ->
                  rotationsPerSecond * 60.0) // 1 Rotation Per Second = 60 Rotations Per Minute
          .toBase(
              rotationsPerMinute ->
                  rotationsPerMinute / 60.0) // 1 Rotation Per Minute = 1/60 Rotations Per Second
          .make();
}
