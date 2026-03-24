// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.climber.ClimberConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.DoubleSupplier;

/**
 * Real IO implementation for the climb subsystem using a Spark Max motor controller.
 *
 * <p>This implementation configures the Spark Max, monitors physical limit switches connected
 * directly to the controller, and implements software limits to prevent driving past the mechanism
 * limits.
 */
public class ClimberIOReal implements ClimberIO {
  private final SparkMax spark = new SparkMax(liftMotorCanId, MotorType.kBrushed);

  // Debouncer to prevent rapidly toggling connection status
  private final Debouncer liftConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  /** Creates a new ClimberIOReal and configures the Spark Max. */
  public ClimberIOReal() {
    var liftCfg = new SparkMaxConfig();
    liftCfg
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) liftMotorCurrentLimit.in(Amps))
        .voltageCompensation(12.0);
    // Configure signal update rates for logging
    liftCfg.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

    // Attempt to configure the Spark Max, retrying if necessary
    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                liftCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  /** Updates hardware inputs, monitors connectivity, and reads limit switch states. */
  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // Safely retrieve telemetry from the motor controller
    sparkStickyFault = false;
    ifOk(
        spark,
        new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
        (vals) -> inputs.appliedVoltage = Volts.of(vals[0] * vals[1]));
    ifOk(spark, spark::getOutputCurrent, (v) -> inputs.appliedCurrent = Amps.of(v));

    // Debounce the connection status to ensure stability
    inputs.connected = liftConnectedDebounce.calculate(!sparkStickyFault);
  }

  /**
   * Sets the lift motor voltage, enforcing limit switch safety checks.
   *
   * @param voltage The requested voltage to apply.
   */
  @Override
  public void setLiftVoltage(Voltage voltage) {
    spark.setVoltage(voltage);
  }
}
