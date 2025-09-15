package frc.robot;

import java.util.List;
import java.util.function.DoubleSupplier;

import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import coppercore.wpilib_interface.Controllers;
import coppercore.wpilib_interface.DriveWithJoysticks;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;

public final class InitBindings {

  private static List<Controllers.Controller> controllers;

  public static void initControllers() {

    Controllers.synced.setFile(
        EnvironmentHandler.getEnvironmentHandler()
            .getEnvironmentPathProvider()
            .resolveReadPath(JsonConstants.operatorConstants.mappingFile));
    Controllers.loadControllers();

    controllers = Controllers.getControllers();
  }

  public static DoubleSupplier getAxis(String command) {
    for (Controllers.Controller controller : controllers) {
      if (controller.hasAxis(command)) {
        return controller.getAxis(command);
      }
    }
    System.out.println("Could not find Axis with command: " + command);
    return () -> 0;
  }

  public static Trigger getButton(String command) {
    for (Controllers.Controller controller : controllers) {
      if (controller.hasButton(command)) {
        return controller.getButton(command);
      }
    }
    System.out.println("Could not find Button with command: " + command);
    return new Trigger(() -> false);
  }

  public static void initDriveBindings(Drive drive) {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive, // type: DriveTemplate
            () -> -getAxis("driveX").getAsDouble(),
            () -> -getAxis("driveY").getAsDouble(),
            () -> getAxis("driveRotation").getAsDouble(),
            JsonConstants.drivetrainConstants.maxLinearSpeed, // type: double (m/s)
            JsonConstants.drivetrainConstants.maxAngularSpeed, // type: double (rad/s)
            JsonConstants.drivetrainConstants.joystickDeadband // type: double
            ));
  }
}
