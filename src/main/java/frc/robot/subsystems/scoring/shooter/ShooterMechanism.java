package frc.robot.subsystems.scoring.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import coppercore.parameter_tools.LoggedTunableNumber;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.scoring.shooter.ShooterIO.ShooterInputs;
import frc.robot.util.AllianceUtil;
import frc.robot.util.GeomUtil;
import java.nio.ByteBuffer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterMechanism {
  /**
   * A set of Speeds for the shooter wheels.
   *
   * <p>This class is StructSerializable so it can be logged by AdvantageKit. It logs its values in
   * RPM.
   */
  public static class ShooterSpeeds implements StructSerializable {
    public final AngularVelocity leftSpeed;
    public final AngularVelocity rightSpeed;

    public static final Struct<ShooterSpeeds> struct =
        new Struct<>() {
          @Override
          public int getSize() {
            return Double.BYTES * 2;
          }

          @Override
          public Class<ShooterSpeeds> getTypeClass() {
            return ShooterSpeeds.class;
          }

          @Override
          public String getTypeName() {
            return "ShooterSpeeds";
          }

          @Override
          public String getSchema() {
            return "double leftRPM;double rightRPM;";
          }

          @Override
          public void pack(ByteBuffer bb, ShooterSpeeds speeds) {
            bb.putDouble(speeds.leftSpeed.in(RPM));
            bb.putDouble(speeds.rightSpeed.in(RPM));
          }

          @Override
          public ShooterSpeeds unpack(ByteBuffer bb) {
            double leftRPM = bb.getDouble();
            double rightRPM = bb.getDouble();

            return new ShooterSpeeds(RPM.of(leftRPM), RPM.of(rightRPM));
          }
        };

    public ShooterSpeeds(AngularVelocity leftSpeed, AngularVelocity rightSpeed) {
      this.leftSpeed = leftSpeed;
      this.rightSpeed = rightSpeed;
    }
  }

  private static final ShooterSpeeds ZERO_SPEEDS =
      new ShooterSpeeds(RotationsPerSecond.zero(), RotationsPerSecond.zero());

  private final ShooterIO leftIO;
  private final ShooterIO rightIO;
  private ShooterInputsAutoLogged leftInputs = new ShooterInputsAutoLogged();
  private ShooterInputsAutoLogged rightInputs = new ShooterInputsAutoLogged();

  /** The last set shooter speeds */
  @AutoLogOutput(key = "scoring/shooter/goalSpeeds")
  private ShooterSpeeds goalSpeeds = ZERO_SPEEDS;

  private enum ShooterOutputMode {
    CLOSED_LOOP,
    VOLTAGE,
    CURRENT,
    STOP,
  }

  @AutoLogOutput(key = "scoring/shooter/outputMode")
  private ShooterOutputMode outputMode = ShooterOutputMode.CLOSED_LOOP;

  /** The states of the shooter. This isn't a state machine because there are only two. */
  private enum ShooterAction {
    STOP,
    WARMUP
  }

  /** The current action of the shooter. */
  private ShooterAction action = ShooterAction.STOP;

  /**
   * Whether or not the Shooter is currently shooting based on the robot's pose
   *
   * <p>When this value is false, the robot falls back to a predefined set of ShooterSpeeds
   */
  @AutoLogOutput(key = "scoring/shooter/poseBasedShooting")
  private boolean poseBasedShooting = false;

  /** Track whether the pose supplier was ever set */
  private boolean poseSupplierInitialized = false;

  private Supplier<Pose2d> poseSupplier = () -> Pose2d.kZero;

  /**
   * Is the shot we're currently warming up for attainable?
   *
   * <p>This value will stop the shooter from being reported as "ready" if it is warmed up for a
   * shot that the robot isn't yet aimed at.
   *
   * <p>If pose-based speeds are enabled and the robot is aimed away from the barge, this value will
   * be false while the robot aims for the closest shot that it could take. Then, once the robot is
   * rotated far enough inward, the shot will become attainable and this value will become true.
   */
  @AutoLogOutput(key = "scoring/shooter/isShotAttainable")
  private boolean isShotAttainable = false;

  // Tunables for Test Mode
  // Tunable gains
  private LoggedTunableNumber shooterKP =
      new LoggedTunableNumber(
          "ShooterTunables/KP", JsonConstants.shooterConstants.baseTalonFXConfigs.Slot0.kP);
  private LoggedTunableNumber shooterKI =
      new LoggedTunableNumber(
          "ShooterTunables/KI", JsonConstants.shooterConstants.baseTalonFXConfigs.Slot0.kI);
  private LoggedTunableNumber shooterKD =
      new LoggedTunableNumber(
          "ShooterTunables/KD", JsonConstants.shooterConstants.baseTalonFXConfigs.Slot0.kD);

  private LoggedTunableNumber shooterKS =
      new LoggedTunableNumber(
          "ShooterTunables/KS", JsonConstants.shooterConstants.baseTalonFXConfigs.Slot0.kS);
  private LoggedTunableNumber shooterKV =
      new LoggedTunableNumber(
          "ShooterTunables/KV", JsonConstants.shooterConstants.baseTalonFXConfigs.Slot0.kV);
  private LoggedTunableNumber shooterKA =
      new LoggedTunableNumber(
          "ShooterTunables/KA", JsonConstants.shooterConstants.baseTalonFXConfigs.Slot0.kA);

  // Tunable profile
  private LoggedTunableNumber shooterMaxAcceleration =
      new LoggedTunableNumber(
          "ShooterTunables/maxAcceleration",
          JsonConstants.shooterConstants.baseTalonFXConfigs.MotionMagic.MotionMagicAcceleration);

  // Tunable outputs
  private LoggedTunableNumber shooterLeftManualVolts =
      new LoggedTunableNumber("ShooterTunables/LeftManualVolts", 0.0);
  private LoggedTunableNumber shooterRightManualVolts =
      new LoggedTunableNumber("ShooterTunables/RightManualVolts", 0.0);

  private LoggedTunableNumber shooterLeftManualAmps =
      new LoggedTunableNumber("ShooterTunables/LeftManualAmps", 0.0);
  private LoggedTunableNumber shooterRightManualAmps =
      new LoggedTunableNumber("ShooterTunables/RightManualAmps", 0.0);

  private LoggedTunableNumber shooterLeftTargetRPM =
      new LoggedTunableNumber("ShooterTunables/LeftTargetRPM", 0.0);
  private LoggedTunableNumber shooterRightTargetRPM =
      new LoggedTunableNumber("ShooterTunables/RightTargetRPM", 0.0);

  public ShooterMechanism(ShooterIO leftShooterIO, ShooterIO rightShooterIO) {
    this.leftIO = leftShooterIO;
    this.rightIO = rightShooterIO;
  }

  /**
   * Set the pose supplier used by the Shooter for RPM calculations and enable pose based shooting.
   *
   * @param newPoseSupplier The new supplier for poses to use in distance calculations
   */
  public void initializePoseSupplier(Supplier<Pose2d> newPoseSupplier) {
    poseSupplier = newPoseSupplier;
    if (!poseSupplierInitialized) {
      poseSupplierInitialized = true;
      poseBasedShooting = true;
    }
  }

  /**
   * Set whether or not the shooter should use pose based shooting
   *
   * <p>This method should be called to enable/disable vision-based shots whenever we gain/lose
   * confidence in vision & odometry
   *
   * <p>If initializePoseSupplier has never been called, poseBasedShootingEnabled will be ignored
   * until it is initialized. This means that, if this method is called with `true`, nothing wil
   * happen until the pose supplier is initialized, after which the shooter will begin using
   * pose-based shooting.
   *
   * @param poseBasedShootingEnabled True if the shooter should calculate its shooter speeds based
   *     on the pose supplier, false if it should fall back to default setpoint
   */
  public void setPoseBasedShootingEnabled(boolean poseBasedShootingEnabled) {
    poseBasedShooting = poseBasedShootingEnabled;
  }

  /**
   * This method should be called in each periodic loop by the ScoringSubsystem. It will NOT run
   * automatically.
   */
  public void periodic() {
    leftIO.updateInputs(leftInputs);
    rightIO.updateInputs(rightInputs);

    Logger.processInputs("scoring/shooter/leftInputs", leftInputs);
    Logger.processInputs("scoring/shooter/rightInputs", rightInputs);

    Logger.recordOutput("scoring/shooter/leftRPM", leftInputs.motorVelocity.in(RPM));
    Logger.recordOutput("scoring/shooter/rightRPM", rightInputs.motorVelocity.in(RPM));

    switch (action) {
      case STOP -> {
        stop();
      }
      case WARMUP -> {
        if (poseSupplierInitialized && poseBasedShooting) {
          ShooterSpeeds speeds = calculatePoseBasedSpeeds();
          runSpeeds(speeds);
        } else {
          // Fall back to default shot
          runSpeeds(JsonConstants.shooterConstants.defaultShot);
        }
      }
    }
  }

  /**
   * This method should be called in each periodic loop by the ScoringSubsystem when the robot is in
   * test mode. It will NOT run automatically.
   */
  public void testPeriodic() {
    switch (TestModeManager.getTestMode()) {
      case ShooterCurrentTuning -> {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (currents) -> {
              leftIO.runOpenLoop(Amps.of(currents[0]));
              rightIO.runOpenLoop(Amps.of(currents[1]));

              outputMode = ShooterOutputMode.CURRENT;
            },
            shooterLeftManualAmps,
            shooterRightManualAmps);
      }

      case ShooterVoltageTuning -> {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (voltages) -> {
              leftIO.runOpenLoop(Volts.of(voltages[0]));
              rightIO.runOpenLoop(Volts.of(voltages[1]));

              outputMode = ShooterOutputMode.VOLTAGE;
            },
            shooterLeftManualVolts,
            shooterRightManualVolts);
      }

      case ShooterClosedLoopTuning -> {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid) -> {
              leftIO.setPID(pid[0], pid[1], pid[2]);
              rightIO.setPID(pid[0], pid[1], pid[2]);
            },
            shooterKP,
            shooterKI,
            shooterKD);
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (ff) -> {
              leftIO.setFFSVA(ff[0], ff[1], ff[2]);
              rightIO.setFFSVA(ff[0], ff[1], ff[2]);
            },
            shooterKS,
            shooterKV,
            shooterKA);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (speeds) -> {
              runSpeeds(new ShooterSpeeds(RPM.of(speeds[0]), RPM.of(speeds[1])));
            },
            shooterLeftTargetRPM,
            shooterRightTargetRPM);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (accel) -> {
              leftIO.setMaxProfileAcceleration(RotationsPerSecondPerSecond.of(accel[0]));
              rightIO.setMaxProfileAcceleration(RotationsPerSecondPerSecond.of(accel[0]));
            },
            shooterMaxAcceleration);
      }

      default -> {}
    }
  }

  /**
   * Run the shooter wheels at a certain set of speeds.
   *
   * <p>This also updates the goal speeds of the shooter, for reference in {@link
   * ShooterMechanism#shooterReady()}
   *
   * @param speeds The set of speeds to run the shooter at
   */
  private void runSpeeds(ShooterSpeeds speeds) {
    leftIO.runSpeed(speeds.leftSpeed);
    rightIO.runSpeed(speeds.rightSpeed);

    outputMode = ShooterOutputMode.CLOSED_LOOP;
  }

  /**
   * Warm up the shooter wheels to score
   *
   * <p>If pose-based shooting is enabled, this will calculate the distance to the barge every cycle
   * to find its target RPM. Otherwise, it will fall back to its default speeds.
   */
  public void warmUp() {
    action = ShooterAction.WARMUP;
  }

  /** Stop the shooter wheels, setting their goal speeds to zero */
  public void stop() {
    leftIO.stop();
    rightIO.stop();

    goalSpeeds = ZERO_SPEEDS;
    outputMode = ShooterOutputMode.STOP;

    action = ShooterAction.STOP;
  }

  /**
   * Calculate ShooterSpeeds based on the robot's pose and the distance -> speeds map. Then, update
   * isShotAttainable based on whether or not the robot is actually in a position to make the
   * closest possible shot on the barge.
   *
   * @return The ShooterSpeeds that the shooters should warm up at.
   */
  private ShooterSpeeds calculatePoseBasedSpeeds() {
    // Calculate distance and use lookup-table
    Pose2d robotPose = poseSupplier.get();

    Pair<Translation2d, Translation2d> bargeSegment;
    if (AllianceUtil.isRed()) {
      bargeSegment = JsonConstants.redFieldLocations.bargeLine;
    } else {
      bargeSegment = JsonConstants.blueFieldLocations.bargeLine;
    }

    // Find the distance to the barge, and whether or not the robot is currently pointed
    // perpendicular to a point that is actually on the barge.
    // We do this by finding the intersection point of the ray pointing out of the side of the
    // robot. If this point falls on the barge line segment, we can shoot at that point.
    // Otherwise, we warm up as if we're aimed at the closest point on the barge line segment
    // to where the robot is aimed, but block its ability to score (make shooterReady return
    // false). This means that as soon as the robot turns back toward the barge, it'll be
    // ready to score.

    // The goal is to find a parameter (t) that gives us the position on the barge we're shooting
    // at. If 0 <= t <= 1, we are aimed at the barge. Otherwise, we're turned too far outward.

    // The direction of the shot to take. This will either be 90 degrees to the left or right
    // of the robot's direction.
    final Rotation2d shotDirection;

    // Keep track of whether we're shooting left or right
    final boolean isShotLeft;

    if (AllianceUtil.isRed()) {
      if (robotPose.getRotation().getDegrees() < 0.0) {
        // Shooting right, shot direction is robot pose turned CW 90 degrees
        shotDirection = robotPose.getRotation().plus(Rotation2d.kCW_90deg);
        Logger.recordOutput("scoring/shooter/poseBasedShotSide", "right");
        isShotLeft = false;
      } else {
        // Shooting left, shot direction is robot pose turned CCW 90 degrees
        shotDirection = robotPose.getRotation().plus(Rotation2d.kCCW_90deg);
        Logger.recordOutput("scoring/shooter/poseBasedShotSide", "left");
        isShotLeft = true;
      }
    } else {
      // Blue's "forward heading" is 180 degrees, so these are reversed from red
      if (robotPose.getRotation().getDegrees() < 0.0) {
        // Shooting left, shot direction is robot pose turned CCW 90 degrees
        shotDirection = robotPose.getRotation().plus(Rotation2d.kCCW_90deg);
        Logger.recordOutput("scoring/shooter/poseBasedShotSide", "left");
        isShotLeft = true;
      } else {
        // Shooting right, shot direction is robot pose turned CW 90 degrees
        shotDirection = robotPose.getRotation().plus(Rotation2d.kCW_90deg);
        Logger.recordOutput("scoring/shooter/poseBasedShotSide", "right");
        isShotLeft = false;
      }
    }

    // The following algorithm is roughly adapted from an algorithm generated by OpenAI
    // ChatGPT
    // Any parts of this code including comments (until the comment denoting its end) not modified
    // after 2025-09-14 are AI
    // generated (or written with AI assistance).

    /*
    Derivation of intersection parameter t (along the segment):

    We want the intersection of a line segment and a ray:

        p0 + t s = r0 + u d

    where
        p0 = segment start
        s  = p1 - p0  (segment vector)
        r0 = ray origin
        d  = (cos θ, sin θ)  (ray direction)
        r  = r0 - p0

    Step 1. Rearrange:
        t s - u d = r

    Step 2. Take 2D cross product with d:
        (t s - u d) × d = r × d

    Step 3. Simplify:
        t (s × d) - u (d × d) = r × d
        t (s × d) = r × d      (since d × d = 0)

    Step 4. Solve for t:
        t = (r × d) / (s × d)

    Similarly, cross with s to solve for u:
        u = (r × s) / (s × d)

    Notes:
        - If s × d = 0, the segment and ray are parallel.
        - Valid intersection requires u ≥ 0 (in front of ray).
        - For t:
            t = 0   → intersection at p0
            t = 1   → intersection at p1
            t < 0   → before segment start
            t > 1   → beyond segment end
    */

    // The following implementation is a hand-translation of chatGPT's version, since the variable
    // names and structure of chatGPT's code were not up to my standards.
    // All of the code that is not geometry (e.g. the decision for what shooter speeds to pick) was
    // handwritten as ChatGPT was not appraised of the entirety of the situation.
    Translation2d segmentVector = bargeSegment.getSecond().minus(bargeSegment.getFirst()); // s
    Translation2d rayDirection =
        new Translation2d(shotDirection.getCos(), shotDirection.getSin()); // d
    // The origin of the ray is the robot's current Translation. Thus r = r0 - p0 = robot
    // translation - barge line start.
    Translation2d r = robotPose.getTranslation().minus(bargeSegment.getFirst());

    double denominator = GeomUtil.cross(segmentVector, rayDirection);

    double shotDistance;

    isShotAttainable = true;

    if (Math.abs(denominator) < 1e-9) {
      // The lines are collinear if s x d = 0

      // Warm up for the furthest shot in the shooter map
      shotDistance = Double.MAX_VALUE;

      // We can't shoot because we're not even pointed at the barge
      isShotAttainable = false;
    } else {
      double t = GeomUtil.cross(r, rayDirection) / denominator; // t = (r x d) / (s x d)

      // Store this value for easier visualization on the logs
      Translation2d unclampedTargetPoint = bargeSegment.getFirst().plus(segmentVector.times(t));

      if (t < 0 || t > 1) {
        // If t is outside of [0, 1] then we're aimed past the edge of the barge and the shot isn't
        // attainable
        isShotAttainable = false;

        // Now clamp t to the nearest point on the barge so we're prepared to turn toward it
        t = Math.min(Math.max(0.0, t), 1.0);
      }
      // END AI-GENERATED CODE

      Translation2d pointToShootAt = bargeSegment.getFirst().plus(segmentVector.times(t));

      // Log trajectories for the planned shots for easier debugging
      Logger.recordOutput(
          "scoring/shooter/unclampedShotTarget",
          new Translation2d[] {robotPose.getTranslation(), unclampedTargetPoint});
      Logger.recordOutput(
          "scoring/shooter/clampedShotTarget",
          new Translation2d[] {robotPose.getTranslation(), pointToShootAt});

      shotDistance = robotPose.getTranslation().getDistance(pointToShootAt);
    }

    Logger.recordOutput("scoring/shooter/unclampedShotDistance", shotDistance);

    if (shotDistance > JsonConstants.shooterConstants.maxShotDistance) {
      isShotAttainable = false;

      shotDistance = JsonConstants.shooterConstants.maxShotDistance;
    } else if (shotDistance < JsonConstants.shooterConstants.minShotDistance) {
      isShotAttainable = false;

      shotDistance = JsonConstants.shooterConstants.minShotDistance;
    }

    Logger.recordOutput("scoring/shooter/clampedShotDistance", shotDistance);

    double closeSpeedRPM = JsonConstants.shooterConstants.distanceToCloseRPM.get(shotDistance);
    double farSpeedRPM = JsonConstants.shooterConstants.distanceToFarRPM.get(shotDistance);

    AngularVelocity leftSpeed;
    AngularVelocity rightSpeed;

    if (isShotLeft) {
      leftSpeed = RPM.of(closeSpeedRPM);
      rightSpeed = RPM.of(farSpeedRPM);
    } else {
      leftSpeed = RPM.of(farSpeedRPM);
      rightSpeed = RPM.of(closeSpeedRPM);
    }

    return new ShooterSpeeds(leftSpeed, rightSpeed);
  }

  /**
   * Checks whether the shooter currently within the error margin of its goal speeds.
   *
   * <p>If the current shot is not attainable, this will return false
   *
   * @return Whether the shooter is currently within the error margin of its goal speeds
   */
  public boolean shooterReady() {
    if (outputMode != ShooterOutputMode.CLOSED_LOOP) {
      return true;
    }

    if (!isShotAttainable) {
      return false;
    }

    boolean leftReady =
        goalSpeeds.leftSpeed.isNear(
            leftInputs.motorVelocity,
            JsonConstants.shooterConstants.shooterVelocityEpsilonFraction);
    boolean rightReady =
        goalSpeeds.rightSpeed.isNear(
            rightInputs.motorVelocity,
            JsonConstants.shooterConstants.shooterVelocityEpsilonFraction);

    Logger.recordOutput("scoring/shooter/leftReady", leftReady);
    Logger.recordOutput("scoring/shooter/rightReady", rightReady);

    boolean shooterReady = leftReady && rightReady;

    Logger.recordOutput("scoring/shooter/shooterReady", shooterReady);

    return shooterReady;
  }

  public final ShooterInputs getLeftInputs() {
    return leftInputs;
  }

  public final ShooterInputs getRightInputs() {
    return rightInputs;
  }
}
