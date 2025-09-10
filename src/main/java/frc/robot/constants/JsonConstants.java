package frc.robot.constants;

import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.subsystems.intake.IntakeConstants;

public class JsonConstants {
  public static void loadConstants() {
    EnvironmentHandler.getEnvironmentHandler(
        Filesystem.getDeployDirectory().toPath().resolve("constants/config.json").toString());
    IntakeConstants.synced.saveData();
    IntakeConstants.Sim.synced.saveData();
    CANConstants.synced.saveData();
    IntakeConstants.synced.loadData();
    IntakeConstants.Sim.synced.loadData();
    CANConstants.synced.loadData();
    FeatureFlags.synced.loadData();

    featureFlags = FeatureFlags.synced.getObject();
  }

  public static FeatureFlags featureFlags;
  public static IntakeConstants intakeConstants;
  public static IntakeConstants.Sim intakeConstantsSim;
  public static CANConstants canConstants;
}
