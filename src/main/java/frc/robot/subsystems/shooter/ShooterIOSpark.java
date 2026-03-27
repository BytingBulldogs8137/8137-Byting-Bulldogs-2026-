// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;

/**
 * This superstructure implementation is for Spark devices. It defaults to brushless control, but
 * can be easily adapted for a brushed motor. One or more Spark Flexes can be used by swapping
 * relevant instances of "SparkMax" with "SparkFlex".
 */
public class ShooterIOSpark implements ShooterIO {
  private final SparkMax feeder = new SparkMax(feederCanId, MotorType.kBrushed);
  private final SparkMax intakeLauncher = new SparkMax(intakeLauncherCanId, MotorType.kBrushless);

  public ShooterIOSpark() {
    var feederConfig = new SparkMaxConfig();
    feederConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(feederCurrentLimit)
        .voltageCompensation(12.0)
        .openLoopRampRate(0.05)
        .inverted(false);
    tryUntilOk(
        feeder,
        5,
        () ->
            feeder.configure(
                feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    var intakeLauncherConfig = new SparkMaxConfig();
    intakeLauncherConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(intakeLauncherCurrentLimit)
        .voltageCompensation(12.0);
    tryUntilOk(
        intakeLauncher,
        5,
        () ->
            intakeLauncher.configure(
                intakeLauncherConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    ifOk(
        feeder,
        new DoubleSupplier[] {feeder::getAppliedOutput, feeder::getBusVoltage},
        (values) -> inputs.feederAppliedVolts = values[0] * values[1]);
    ifOk(feeder, feeder::getOutputCurrent, (value) -> inputs.feederCurrentAmps = value);

    ifOk(
        intakeLauncher,
        new DoubleSupplier[] {intakeLauncher::getAppliedOutput, intakeLauncher::getBusVoltage},
        (values) -> inputs.intakeLauncherAppliedVolts = values[0] * values[1]);
    ifOk(
        intakeLauncher,
        intakeLauncher::getOutputCurrent,
        (value) -> inputs.intakeLauncherCurrentAmps = value);
  }

  @Override
  public void setFeederVoltage(double volts) {
    System.out.println(volts);
    feeder.setVoltage(volts);
  }

  @Override
  public void setIntakeLauncherVoltage(double volts) {
    intakeLauncher.setVoltage(volts);
  }
}
