package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private final Alert disconnectedAlert = new Alert("Climb IO Disconnected", AlertType.kWarning);

  public Climb(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    disconnectedAlert.set(!(inputs.liftConnected));
  }

  @Override
  public void simulationPeriodic() {}

  private void startLiftUp() {
    io.setLiftOpenLoop(defaultLiftVoltage);
  }

  private void startLiftDown() {
    io.setLiftOpenLoop(-defaultLiftVoltage);
  }

  private void stopLift() {
    io.setLiftOpenLoop(0.0);
  }

  public void stop() {
    stopLift();
  }

  public Command liftUpCommand() {
    return Commands.race(
        Commands.runEnd(this::startLiftUp, this::stopLift, this),
        Commands.waitUntil(() -> inputs.liftUpperLimit || !inputs.liftConnected));
  }

  public Command liftDownCommand() {
    return Commands.race(
        Commands.runEnd(this::startLiftDown, this::stopLift, this),
        Commands.waitUntil(() -> inputs.liftLowerLimit || !inputs.liftConnected));
  }
}
