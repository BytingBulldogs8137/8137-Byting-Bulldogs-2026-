package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;

public class ClimbUpCommand extends Command {
  private final Climber climber;

  public ClimbUpCommand(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void execute() {
    climber.climbUp();
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  // @Override
  // public boolean isFinished() {
  //     return climber.isAtTop();
  // }
}
