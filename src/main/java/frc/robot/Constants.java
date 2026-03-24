// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int driverControllerPort = 0; //declares USB port 0 in driver station as driver controller
    public static final int operatorControllerPort = 1; //declares USB port 1 in driver station as operator/secondary controller
    public static final double DeadBand = 0.05; //controller deadband

    public static final double maxSpeed = Units.feetToMeters(10);

    public static final int Launcher = 10; //launcher motor CAN ID
    public static final int Intake = 9; //Intake motor CAN ID
  }

  public static final Mode simMode = Mode.SIM; //creates robot sim
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /** Total length of the field. */
  public static final Distance fieldLength = Inches.of(651.25);
  /** Total width of the field. */
  public static final Distance fieldWidth = Inches.of(317.5);
}
