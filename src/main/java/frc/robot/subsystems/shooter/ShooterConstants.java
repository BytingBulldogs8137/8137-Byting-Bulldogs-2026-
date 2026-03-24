// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter; //shooter subsystem import

public class ShooterConstants {
  public static final int feederCanId = 10; //feeder(inside) roller
  public static final double feederMotorReduction = 1.0; //reduction of speed of feeder motor
  public static final int feederCurrentLimit = 60; // limit of what the feeder motor can run in RPM

  public static final int intakeLauncherCanId = 9; //front roller
  public static final double intakeLauncherMotorReduction = 1.0; //reduction of speed of intake motor
  public static final int intakeLauncherCurrentLimit = 60; //limit of what the intake motor can run in RPM

  public static final double intakingFeederVoltage = -12.0; //feeder roller voltage when intake command takes place
  public static final double launchingFeederVoltage = 9.0; //feeder roller voltage when shooting command takes place
  public static final double launchingLauncherVoltage = 9.5; //changed to fit right speed of motor. default was 10.6
  public static final double spinUpFeederVoltage = -6.0; //flywheel voltage when shooting command takes place
  public static final double spinUpSeconds = 1.0; //seconds it takes for the flywheel to spin once command begins
}
