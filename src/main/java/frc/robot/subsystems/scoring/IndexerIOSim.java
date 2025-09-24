package frc.robot.subsystems.scoring;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.scoring.IndexerIO.IndexerInputs;

public class IndexerIOSim implements IndexerIO {
  TalonFXSimState indexerMotorSimState = new TalonFXSimState(JsonConstants.);
}
