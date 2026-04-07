package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private static Climber climber = null;

  // Solenoid that controls the climber. Named clearly to avoid confusion with the class.
  private final Solenoid climberSolenoid =
      new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.climberSolenoidChannel);

  public static Climber getInstance() {
    if (climber == null) {
      climber = new Climber();
    }
    return climber;
  }

  private Climber() {
    // Initialize hardware and state here
  }
  /**
   * Deploy the climber solenoid (set it up). This is an instant command so the solenoid will remain
   * in that state after the command completes.
   */
  public Command Up() {
    return runOnce(() -> climberSolenoid.set(true));
  }

  /** Retract the climber solenoid (set it down). Instant command so it stays until changed. */
  public Command Down() {
    return runOnce(() -> climberSolenoid.set(false));
  }

  /** Toggle the climber solenoid state (useful for button toggle bindings). */
  public Command toggle() {
    return runOnce(() -> climberSolenoid.set(!climberSolenoid.get()));
  }

  // Legacy/explicit names kept as instant commands for compatibility with any existing bindings.
  public Command StartUp() {
    return Up();
  }

  public Command StopUp() {
    return Down();
  }

  public Command StartDown() {
    return Down();
  }

  public Command StopDown() {
    return Up();
  }
}
