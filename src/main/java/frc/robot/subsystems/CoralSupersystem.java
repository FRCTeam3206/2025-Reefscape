// Also coral wheels in this system
// Bigger safe angle (for later)

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GameConstants;

@Logged
public class CoralSupersystem {
  /* All of these subsystems should have a command that sets them to each desired position
   * and ends once that position is reached (with another method with "continuous" in its name that
   * allows you to run it without stopping once the goal is reached), and then it should continue to
   * maintain this position after the command stops by setting power in periodic accoding to a saved goal.
   */
  private final Elevator m_elevator = new Elevator();
  private final Arm m_arm = new Arm();
  private final Wrist m_wrist = new Wrist();
  private final CoralIntake m_coralOmnis = new CoralIntake();

  public CoralSupersystem() {}

  /**
   * It can hit things if the wrist is vertical, so we can move the arm first to make sure it's
   * safe.
   */
  public Command moveWristVertical() {
    return (m_arm.toHorizontal().andThen(m_wrist.toVerticalContinuous()))
        .until(() -> m_wrist.isVertical().getAsBoolean());
  }

  /**
   * If we're moving the wrist horizontal, it won't hit things, but we might want to make sure it's
   * horizontal before moving the arm.
   */
  public Command moveWristHorizontal() {
    return m_wrist.toHorizontalContinuous().until(() -> m_wrist.isHorizontal().getAsBoolean());
  }

  /** Move the arm, wrist, and elevator so that the mechanism is in the stored position. */
  public Command positionStore() {
    return moveWristHorizontal()
        .andThen(m_arm.toStored().until(() -> m_arm.aboveHorizontal().getAsBoolean()))
        .andThen(m_elevator.toStored().alongWith(m_arm.toStored()));
  }

  /** Move to intake from the floor. */
  public Command positionFloorIntake() {
    return moveWristHorizontal().andThen(m_elevator.toFloorIntake()).andThen(m_arm.toFloorIntake());
  }

  /** Move to intake from the feeder station. */
  public Command positionFeederStation() {
    return moveWristHorizontal().andThen(m_elevator.toFeeder().alongWith(m_arm.toFeeder()));
  }

  /** Position to place coral on any level, including 1. */
  public Command positionReef(GameConstants.ReefLevels level) {
    return moveWristVertical().andThen(m_elevator.toBranch(level).andThen(m_arm.toBranch(level)));
  }

  public Command floorIntake() {
    return positionFloorIntake().andThen(m_coralOmnis.intakeUntilSuccessCommand());
  }

  public Command feederStation() {
    return positionFeederStation().andThen(m_coralOmnis.intakeUntilSuccessCommand());
  }

  public Command scoreCoral(GameConstants.ReefLevels level) {
    return positionReef(level).andThen(m_coralOmnis.scoreCommand());
  }

  public CoralIntake getOmnisSubsystem() {
    return m_coralOmnis;
  }
}
