package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public boolean liftConnected = false;
    public double liftAppliedVolts = 0.0;
    public double liftCurrentAmps = 0.0;

    public boolean liftLowerLimit = false;
    public boolean liftUpperLimit = false;
  }

  public void updateInputs(ClimbIOInputs inputs);

  public void setLiftOpenLoop(double volts);
}
