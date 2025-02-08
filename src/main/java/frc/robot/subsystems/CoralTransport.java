package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

public class CoralTransport {
  /* All of these subsystems should have a command that sets them to each desired position
   * and ends once that position is reached (with another method with "continuous" in its name that
   * allows you to run it without stopping once the goal is reached), and then it should continue to
   * maintain this position after the command stops by setting power in periodic accoding to a saved goal.
  */
  private final Elevator m_elevator = new Elevator();
  private final Arm m_arm = new Arm();
  private final Wrist m_wrist = new Wrist();

  public CoralTransport() {}

  /** Move the arm and wrist to the default position before moving the elevator, to avoid hitting something. */
  public Command moveSafeHorizontalWrist() {
    return ((m_arm.toDefault())
      .andThen(m_wrist.toHorizontalContinuous()))
      .until(() -> m_wrist.isHorizontal());
  }

  /* If wrist is already vertical, we are good. If wrist is horizontal, we should first move the arm straight then make wrist vertical. */
  public Command moveSafeVerticalWrist() {
    return (m_arm.toDefault()
      .andThen(m_wrist.toVerticalContinuous()))
      .until(() -> m_wrist.isVertical());
  }

  /** Move the arm, wrist, and elevator so that the mechanism is in the stored position. */
  public Command storeCommand() {
    return moveSafeHorizontalWrist()
      .andThen(m_elevator.toStored())
      .alongWith(m_arm.toStored());
  }

  /** Move to intake from the floor. */
  public Command floorIntakeCommand() {
    return moveSafeHorizontalWrist()
      .andThen(m_elevator.toFloorIntake()
      .alongWith(m_arm.toFloorIntake()));
      // The wrist should be in the default position for intaking, which was already achieved by the first command.
  }

  /** Move to intake from the feeder station. */
  public Command feederStationCommand() {
    return moveSafeVerticalWrist() // Verify whether it's horizontal or vertical for this; I've heard both.
      .andThen(m_elevator.toFeeder()
      .alongWith(m_arm.toFeeder()));
  }

  public Command placeCoralLow() {
    return moveSafeHorizontalWrist()
      .andThen(m_elevator.toLowCoral()
      .alongWith(m_arm.toLowCoral()));
  }

  /**
   * @param level This MUST be 2, 3, or 4
   */
  public Command placeCoralBranch(int level) {
    return moveSafeVerticalWrist()
      .andThen(m_elevator.toBranch(level)
      .andThen(m_arm.toBranch(level)));
  }
}
