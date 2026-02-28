package frc.robot.subsystems.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  // Motor controller for the climber
  private final SparkMax climberMotor;

  // Limit switches for safety
  // private final DigitalInput topLimitSwitch;
  // private final DigitalInput bottomLimitSwitch;

  // Motor configuration constants
  private static final int CLIMBER_MOTOR_ID = 15; // Change to your CAN ID
  // private static final int TOP_LIMIT_CHANNEL = 0; // DIO port
  // private static final int BOTTOM_LIMIT_CHANNEL = 1; // DIO port
  private static final double CLIMB_SPEED = 0.8; // 80% power

  public Climber() {
    // climberMotor = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushless);
    climberMotor = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .voltageCompensation(12.0);

    // Persist parameters to retain configuration in the event of a power cycle
    climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // topLimitSwitch = new DigitalInput(TOP_LIMIT_CHANNEL);
    // bottomLimitSwitch = new DigitalInput(BOTTOM_LIMIT_CHANNEL);
  }

  /** Raise the climber */
  public void climbUp() {
    // if (!isAtTop()) {
    climberMotor.set(CLIMB_SPEED);
    // } else {
    // stop();
    // }
  }

  /** Lower the climber */
  public void climbDown() {
    // if (!isAtBottom()) {
    climberMotor.set(-CLIMB_SPEED);
    // } else {
    // stop();
    // }
  }

  /** Stop the climber motor */
  public void stop() {
    climberMotor.stopMotor();
  }

  /** Check if top limit switch is triggered */
  // public boolean isAtTop() {
  //     return !topLimitSwitch.get(); // Assuming NC wiring
  // }

  /** Check if bottom limit switch is triggered */
  // public boolean isAtBottom() {
  //     return !bottomLimitSwitch.get(); // Assuming NC wiring
  // }

  // @Override
  // public void periodic() {
  //     // Optional: log status to SmartDashboard
  //     // SmartDashboard.putBoolean("Climber Top", isAtTop());
  //     // SmartDashboard.putBoolean("Climber Bottom", isAtBottom());
  // }
}
