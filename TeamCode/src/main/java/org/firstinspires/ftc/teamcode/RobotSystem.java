package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotAutoDriveByGyro_Linear;
import org.firstinspires.ftc.teamcode.Subsystems.CVSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.MovementSubsystem;

public class RobotSystem {

    public MovementSubsystem drive;
    public CVSubsystem cv;

    public HardwareRobot hardwareRobot;

    public RobotSystem(HardwareMap hardware){

        hardwareRobot = new HardwareRobot(hardware);
        drive = new MovementSubsystem(hardwareRobot.leftFront, hardwareRobot.rightFront, hardwareRobot.leftBack, hardwareRobot.rightBack);

    }

}
