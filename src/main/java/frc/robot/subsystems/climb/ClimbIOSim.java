package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.liftGearbox;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimbIOSim implements ClimbIO {
  private final DCMotorSim liftSim;

  private double liftAppliedVolts = 0.0;

  public ClimbIOSim() {
    liftSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(liftGearbox, 0.02, 1.0), liftGearbox);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    liftSim.setInputVoltage(MathUtil.clamp(liftAppliedVolts, -12.0, 12.0));
    liftSim.update(0.02);

    inputs.liftConnected = true;
    inputs.liftAppliedVolts = liftAppliedVolts;
    inputs.liftCurrentAmps = Math.abs(liftSim.getCurrentDrawAmps());

    inputs.liftLowerLimit =
        liftAppliedVolts < 0.0 && Math.abs(liftSim.getAngularPositionRad()) > 10.0;
    inputs.liftUpperLimit =
        liftAppliedVolts > 0.0 && Math.abs(liftSim.getAngularPositionRad()) > 10.0;
  }

  @Override
  public void setLiftOpenLoop(double volts) {
    liftAppliedVolts = volts;
  }
}
