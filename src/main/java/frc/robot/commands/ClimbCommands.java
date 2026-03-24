package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;

/**
 * Factory class for creating commands related to the climbing subsystem.
 *
 * <p>Uses WPILib's Command composition factory methods to create safe, race-conditioned commands
 * that stop when limit switches are triggered.
 */
public class ClimbCommands {
  /**
   * Creates a command to lift the robot up until the upper limit switch is triggered.
   *
   * @param climb The climbing subsystem.
   * @return A command that runs the lift upward until the upper limit is reached.
   */
  public static Command liftUp(Climber climber) {  //command to lift elevator
    return Commands.runEnd(climber::startLiftUp, climber::stopLift, climber); //ends command when executed so
  }

  /**
   * Creates a command to lower the robot down until the lower limit switch is triggered.
   *
   * @param climb The climbing subsystem.
   * @return A command that runs the lift downward until the lower limit is reached.
   */
  public static Command liftDown(Climber climber) { //command to lower elevator
    return Commands.runEnd(climber::startLiftDown, climber::stopLift, climber); //ends command when executed so
  }
}
