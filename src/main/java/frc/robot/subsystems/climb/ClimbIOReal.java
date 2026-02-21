package frc.robot.subsystems.climb;

import static frc.robot.subsystems.climb.ClimbConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClimbIOReal implements ClimbIO {
  private final SparkMax liftA = new SparkMax(liftMotorCanId, MotorType.kBrushless);

  private final DigitalInput liftLowerLimit = new DigitalInput(liftLowerLimitDio);
  private final DigitalInput liftUpperLimit = new DigitalInput(liftUpperLimitDio);

  private final Debouncer liftConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private boolean sparkStickyFault = false;

  public ClimbIOReal() {
    var liftCfg = new SparkMaxConfig();
    liftCfg
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(liftMotorCurrentLimit)
        .voltageCompensation(12.0);

    liftCfg.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    tryUntilOk(
        liftA,
        5,
        () ->
            liftA.configure(
                liftCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        liftA,
        new java.util.function.DoubleSupplier[] {liftA::getAppliedOutput, liftA::getBusVoltage},
        (vals) -> inputs.liftAppliedVolts = vals[0] * vals[1]);
    ifOk(liftA, liftA::getOutputCurrent, (v) -> inputs.liftCurrentAmps = v);
    inputs.liftConnected = liftConnectedDebounce.calculate(!sparkStickyFault);

    inputs.liftLowerLimit = !liftLowerLimit.get();
    inputs.liftUpperLimit = !liftUpperLimit.get();
  }

  public void setLiftOpenLoop(double volts) {
    boolean atUpper = !liftUpperLimit.get();
    boolean atLower = !liftLowerLimit.get();
    if ((volts > 0.0 && atUpper) || (volts < 0.0 && atLower)) {
      liftA.setVoltage(0.0);
    } else {
      liftA.setVoltage(volts);
    }
  }
}
