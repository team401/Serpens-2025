package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class TestModeManager {
  public enum TestMode {
    None, // No test mode selected
    ShooterVoltageTuning("Shooter Voltage Open Loop Tuning"),
    ShooterCurrentTuning("Shooter Current Open Loop Tuning"),
    ShooterClosedLoopTuning("Shooter Closed Loop Tuning");

    private final String description;

    TestMode(String description) {
      this.description = description;
    }

    TestMode() {
      this.description = name();
    }
  }

  // This class should not be instantiated
  private TestModeManager() {}

  private static SendableChooser<TestMode> testModeChooser = null;

  /**
   * Initialize the Test Mode Chooser.
   *
   * <p>Should be called by the RobotContainer at some point during initialization
   */
  public static void init() {
    if (testModeChooser != null) {
      Exception e = new Exception("TestModeManager.init() called more than once!");
      e.printStackTrace();
      return;
    }

    testModeChooser = new SendableChooser<>();

    for (TestMode mode : TestMode.values()) {
      if (mode == TestMode.None) {
        testModeChooser.setDefaultOption(mode.description, mode);
      } else {
        testModeChooser.addOption(mode.description, mode);
      }
    }

    SmartDashboard.putData("Test Mode Selector", testModeChooser);
  }

  /**
   * Return the current Test Mode from the Test Mode Chooser.
   *
   * <p>If the TestModeManager has not been initialized or the robot is not in Test mode, this will
   * return TestMode.None
   */
  public static TestMode getTestMode() {
    if (testModeChooser == null || !DriverStation.isTest()) {
      return TestMode.None;
    }

    return testModeChooser.getSelected();
  }
}
