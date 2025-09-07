package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CV.MotifPipeline;
import org.firstinspires.ftc.teamcode.Subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

import java.util.Optional;

public class RobotSystem {
    public final CVSubsystem cv;
    public final HardwareRobot hardwareRobot;
    public final DriveSubsystem drive;
    public final LinearOpMode opMode;
    public RobotSystem(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.hardwareRobot = new HardwareRobot(hardwareMap);
        this.cv = new CVSubsystem(hardwareRobot.cameraName, opMode, new MotifPipeline());
        this.drive = new DriveSubsystem(
          hardwareRobot.leftFront,
          hardwareRobot.rightFront,
          hardwareRobot.leftBack,
          hardwareRobot.leftFront
        );
        this.opMode = opMode;
    }
}
