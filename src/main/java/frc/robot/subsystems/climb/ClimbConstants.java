package frc.robot.subsystems.climb;

import edu.wpi.first.math.system.plant.DCMotor;

public class ClimbConstants {
  public static final int liftMotorCanId = 14;

  public static final int liftLowerLimitDio = 0;
  public static final int liftUpperLimitDio = 1;

  public static final int liftMotorCurrentLimit = 30;
  public static final DCMotor liftGearbox = DCMotor.getNEO(1);

  public static final double defaultLiftVoltage = 6.0;
}
