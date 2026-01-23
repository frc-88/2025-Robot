package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class QuestNavTJ2 extends SubsystemBase {

  public QuestNav quest = new QuestNav();
  public Pose3d lastPose = new Pose3d();

  Transform3d ROBOT_TO_QUEST =
      new Transform3d(
          Units.inchesToMeters(0.5),
          Units.inchesToMeters(7.5),
          Units.inchesToMeters(11),
          new Rotation3d(0, 0, Math.PI / 2.0));

  public QuestNavTJ2() {}

  public Pose3d getPose() {
    PoseFrame[] poseFrames = quest.getAllUnreadPoseFrames();

    if (poseFrames.length > 0) {
      lastPose =
          poseFrames[poseFrames.length - 1].questPose3d().transformBy(ROBOT_TO_QUEST.inverse());
      return lastPose;
    }

    return lastPose;
  }

  public void resetPose(Supplier<Pose3d> pose) {
    Logger.recordOutput("QuestNav/Reset", true);
    quest.setPose(pose.get().transformBy(ROBOT_TO_QUEST));
  }

  public Command resetQuestPose(Supplier<Pose3d> pose) {
    return new InstantCommand(
        () -> {
          for (int i = 0; i < 10; i++) {
            System.out.println("GETTING RUN !! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !");
          }

          resetPose(pose);
        },
        this);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("QuestNav/Pose", getPose());
    Logger.recordOutput("QuestNav/Battery", quest.getBatteryPercent().orElse(0));
    Logger.recordOutput("QuestNav/IsConnected", quest.isConnected());
    quest.commandPeriodic();
  }
}
