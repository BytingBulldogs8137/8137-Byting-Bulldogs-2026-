// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  /** Set the rollers to the values for intaking. */
  public Command intake() {
    return runEnd(
        () -> {
          io.setFeederVoltage(intakingFeederVoltage);
          io.setIntakeLauncherVoltage(intakingFeederVoltage);
        },
        () -> {
          io.setFeederVoltage(0.0);
          io.setIntakeLauncherVoltage(0.0);
        });
  }

  /** Set the rollers to the values for ejecting fuel out the intake. */
  public Command eject() {
    return runEnd(
        () -> {
          io.setFeederVoltage(-intakingFeederVoltage);
          io.setIntakeLauncherVoltage(-intakingFeederVoltage);
        },
        () -> {
          io.setFeederVoltage(0.0);
          io.setIntakeLauncherVoltage(0.0);
        });
  }

  /** Set the rollers to the values for launching. Spins up before feeding fuel. */
  public Command launch() {
    return run(() -> {
          io.setFeederVoltage(spinUpFeederVoltage);
          io.setIntakeLauncherVoltage(launchingLauncherVoltage);
        })
        .withTimeout(spinUpSeconds)
        .andThen(
            run(
                () -> {
                  io.setFeederVoltage(launchingFeederVoltage);
                  io.setIntakeLauncherVoltage(launchingLauncherVoltage);
                }))
        .finallyDo(
            () -> {
              io.setFeederVoltage(0.0);
              io.setIntakeLauncherVoltage(0.0);
            });
  }
}
