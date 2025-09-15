package frc.robot.constants;

import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;

public final class OperatorConstants {
  @JSONExclude
  public static final JSONSync<OperatorConstants> synced =
      new JSONSync<OperatorConstants>(
          new OperatorConstants(),
          "OperatorConstants.json",
          JsonConstants.getPathProvider(),
          new JSONSyncConfigBuilder().build());

  public String mappingFile = "controllers.json";
}
