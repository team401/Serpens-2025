package frc.robot.constants;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;

public class CANConstants {
  @JSONExclude
  public static final JSONSync<CANConstants> synced =
      new JSONSync<CANConstants>(
          new CANConstants(),
          "CANConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public final Integer shooterLeftMotorID = 13; // TODO: Real motor IDs
  public final Integer shooterRightMotorID = 14;

  public final Frequency updateFrequency = Hertz.of(50.0);

  public final Time deviceConnectedDebounceTime = Seconds.of(0.5);
}
