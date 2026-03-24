// Copyright (c) 2026 FRC Team 4533 (Phoenix)
// Derived from the AdvantageKit framework by Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for the robot's climb mechanism.
 *
 * <p>Handles controlling the lift motor voltage and monitoring limit switches to prevent
 * over-extension or damage to the mechanism.
 */
public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private final Alert disconnectedAlert = new Alert("Climber IO disconnected", AlertType.kWarning);

  /**
   * Creates a new Climb subsystem.
   *
   * @param io The abstraction layer for the climb hardware.
   */
  public Climber(ClimberIO io) {
    this.io = io;
  }

  /** Updates hardware inputs, logs data, and updates status alerts. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
    disconnectedAlert.set(!inputs.connected);
  }

  @Override
  public void simulationPeriodic() {}

  /**
   * Moves the lift mechanism up at the default voltage.
   *
   * @return
   */
  public void startLiftUp() {
    io.setLiftVoltage(defaultLiftVoltage);
  }

  /** Moves the lift mechanism down at the default voltage. */
  public void startLiftDown() {
    io.setLiftVoltage(defaultLiftVoltage.unaryMinus());
  }

  /** Stops the lift mechanism. */
  public void stopLift() {
    io.setLiftVoltage(Volts.of(0.0));
  }
}
