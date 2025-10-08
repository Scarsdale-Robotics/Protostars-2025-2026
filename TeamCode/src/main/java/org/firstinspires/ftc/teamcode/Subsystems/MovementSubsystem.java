package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class MovementSubsystem extends SubsystemBase {

    public Motor frontLeftMotor;
    public Motor frontRightMotor;
    public Motor backLeftMotor;
    public Motor backRightMotor;


    public MovementSubsystem(Motor frontLeftMotor,
                             Motor frontRightMotor,
                             Motor backLeftMotor,
                             Motor backRightMotor)
    {
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        this.motorControl = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
    }
    public static MecanumDrive motorControl;


    double speed;


    public void driveRobotCentric (double forward, double strafe, double turn) {

        motorControl.driveRobotCentric(strafe,forward,turn);
        //frontLeftMotor.setTargetDistance(forward + strafe + turn);
        //frontRightMotor.setTargetDistance(forward - strafe  - turn);
        //backLeftMotor.setTargetDistance(forward - strafe + turn);
        //backRightMotor.setTargetDistance(forward + strafe -turn);


    }




}
