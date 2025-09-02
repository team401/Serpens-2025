package frc.robot.subsystems.scoring; 

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations; 
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.units.AngularAccelerationUnit; 
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog 
  public static class IndexerInputs {
  /**
    * Whether or not the indexer CANcoder is connected
    *
    *<p> This value is determined by whether or not refreshing the Position status signal returns an OK signal
    */
    
  public boolean isIndexerEncoderConnected = false; 

  /** Current indexer position as reported by the CANcoder */
  public MutAngle indexerPosition = Rotations.mutable(0.0);

  /** Current velocity of the indexer as reported by the CANcoder */
  public MutAngularVelocity indexerVelocity = RotationsPerSecond.mutable(0.0);

  /** Goal angle of the indexer, as seen by the indexer CANcoder */
  public MutAngle indexerGoalPosition = Rotations.mutable(0.0);

  /** The setpoint target position from Motion Magic Expo */
  public MutAngle indexerSetpointPosition = Rotations.mutable(0.0);

  /** The target angular velocity of the indexer */
  public MutAngularVelocity indexerTargetVelocity = RotationsPerSecond.mutable(0.0);

  /** Supply current of the indexer motor */ 
  public MutCurrent indexerSupplyCurrent = Amps.mutable(0.0);

  /** Stator current of the indexer motor */
  public MutCurrent indexerStatorCurrent = Amps.mutable(0.0);
}

@AutoLog
public static class IndexerInputs {
  /**
  * closed-loop output of the indexer controller. this value isn't a unit because Phoenix 6 
  * doesn't use a unit for this value (as it can be a Voltage or a Current depending on whether or not FOC is used) 
  */
  public double indexerInput = 0.0; 

}
 /**
  * Updates a IndexerInputs with the current information from sensors and motors
  *
  * <p> Should be called by the IndexerMechanism periodically 
  */
