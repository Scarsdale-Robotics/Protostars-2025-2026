package org.firstinspires.ftc.teamcode.RoadRunner.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This routine is designed to calculate the maximum angular velocity your bot can achieve under load.
 * <p>
 * Upon pressing start, your bot will turn at max power for RUNTIME seconds.
 * <p>
 * Further fine tuning of MAX_ANG_VEL may be desired.
 */

@Config
@Autonomous(group = "drive")
public class MaxAngularVeloTuner extends LinearOpMode {
    public static double RUNTIME = 4.0;

    private ElapsedTime timer;
    private double maxAngVelocity = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
    }
}
