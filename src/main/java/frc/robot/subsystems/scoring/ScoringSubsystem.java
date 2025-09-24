package frc.robot.subsystems.scoring;

import coppercore.wpilib_interface.MonitoredSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.scoring.shooter.ShooterMechanism;
import java.util.Optional;
import java.util.function.Supplier;

public class ScoringSubsystem extends MonitoredSubsystem {
  private static Optional<ScoringSubsystem> instance = Optional.empty();

  private final Optional<IndexerMechanism> optionalIndexer;
  private final Optional<ShooterMechanism> optionalShooter;

  /**
   * Construct a new ScoringSubsystem
   *
   * <p>This constructor is private so that users are forced to use {@link ScoringSubsystem#create}
   *
   * @param indexer The IndexerMechanism instance to use
   * @param shooter The ShooterMechanism instance to use
   */
  private ScoringSubsystem(Optional<IndexerMechanism> indexer, Optional<ShooterMechanism> shooter) {
    this.optionalIndexer = indexer;
    this.optionalShooter = shooter;
  }

  // Create method architecture suggested by OpenAI ChatGPT, although no generated code has been
  // used here.
  /**
   * Create a new ScoringSubsystem, returning the instance created and updating the static instance
   * field.
   *
   * <p>This method exists because updating the instance field with a {@link java.util.Optional}
   * created from `this` would leak a partially initialized object in the constructor. Instead, this
   * method creates the object and <i>then</i> updates the instance by creating an Optional.
   *
   * @param indexer The IndexerMechanism instance to use
   * @param shooter The ShooterMechanism instance to use
   * @return The newly created ScoringSubsystem instance
   */
  public static ScoringSubsystem create(
      Optional<IndexerMechanism> indexer, Optional<ShooterMechanism> shooter) {
    if (instance.isPresent()) {
      throw new Error("ScoringSubsystem was created more than once.");
    }

    ScoringSubsystem createdInstance = new ScoringSubsystem(indexer, shooter);

    instance = Optional.of(createdInstance);

    return createdInstance;
  }

  /**
   * Initialize the Shooter mechanism's drive pose supplier to use for pose-based shots
   *
   * <p>If shooter isn't run, this method is a no-op
   *
   * @param newPoseSupplier A Supplier for a Pose2d that supplies the drivetrain's current odometry
   *     pose
   */
  public void initializeShooterPoseSupplier(Supplier<Pose2d> newPoseSupplier) {
    optionalShooter.ifPresent(shooter -> shooter.initializePoseSupplier(newPoseSupplier));
  }

  /**
   * Get an Optional containing the current ScoringSubsystem instance.
   *
   * <p>If the ScoringSubsystem has not been instantiated, this will be an empty optional
   *
   * @return An Optional that will contain the ScoringSubsystem if it has been instantiated
   */
  public static Optional<ScoringSubsystem> getInstance() {
    return instance;
  }

  @Override
  public void monitoredPeriodic() {
    optionalIndexer.ifPresent(indexer -> indexer.periodic());
    optionalShooter.ifPresent(shooter -> shooter.periodic());
  }

  public void testPeriodic() {
    optionalShooter.ifPresent(shooter -> shooter.testPeriodic());
  }

  /**
   * Warm up the shooter
   *
   * <p>This method exists to give bindings a temporary way to make the shooter warm up before the
   * state machine is implemented.
   *
   * <p>If the shooter isn't enabled in ScoringFeatureFlags, this is a no-op
   */
  public void tempWarmup() {
    optionalShooter.ifPresent(shooter -> shooter.warmUp());
  }

  /**
   * Stop the shooter
   *
   * <p>This method exists to give bindings a temporary way to make the shooter stop warming up
   *
   * <p>If the shooter isn't enabled in ScoringFeatureFlags, this is a no-op before the state
   * machine is implemented.
   */
  public void tempStopShooter() {
    optionalShooter.ifPresent(shooter -> shooter.stop());
  }
}
