package frc.robot;

import frc.robot.constants.ModeConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.scoring.IndexerMechanism;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.shooter.ShooterIO;
import frc.robot.subsystems.scoring.shooter.ShooterIOSim;
import frc.robot.subsystems.scoring.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.scoring.shooter.ShooterMechanism;

public final class InitSubsystems {
  public static Drive initDrive() {
    return switch (ModeConstants.CURRENT_MODE) {
      case REAL ->
      // Real robot, instantiate hardware IO implementations
      new Drive(
          new GyroIOPigeon2(),
          new ModuleIOTalonFX(TunerConstants.FrontLeft),
          new ModuleIOTalonFX(TunerConstants.FrontRight),
          new ModuleIOTalonFX(TunerConstants.BackLeft),
          new ModuleIOTalonFX(TunerConstants.BackRight));

      case SIM ->
      // Sim robot, instantiate physics sim IO implementations
      new Drive(
          new GyroIO() {},
          new ModuleIOSim(TunerConstants.FrontLeft),
          new ModuleIOSim(TunerConstants.FrontRight),
          new ModuleIOSim(TunerConstants.BackLeft),
          new ModuleIOSim(TunerConstants.BackRight));

      default ->
      // Replayed robot, disable IO implementations
      new Drive(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {});
    };
  }

  public static ScoringSubsystem initScoring() {
    return switch (ModeConstants.CURRENT_MODE) {
      case REAL ->
      // Real robot, instantiate hardware IO implementations
      ScoringSubsystem.create(new IndexerMechanism(), new ShooterMechanism(new ShooterIOTalonFX()));

      case SIM ->
      // Sim robot, instantiate physics sim IO implementations
      ScoringSubsystem.create(new IndexerMechanism(), new ShooterMechanism(new ShooterIOSim()));

      default ->
      // Replayed robot, disable IO implementations
      ScoringSubsystem.create(new IndexerMechanism(), new ShooterMechanism(new ShooterIO() {}));
    };
  }
}
