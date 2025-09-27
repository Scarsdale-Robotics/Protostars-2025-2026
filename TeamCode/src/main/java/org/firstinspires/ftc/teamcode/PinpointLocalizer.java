package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PinpointLocalizer implements Localizer {
    private final GoBildaPinpointDriver pinpoint;

    public PinpointLocalizer(GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return new Pose2d(
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(AngleUnit.RADIANS)
        );
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading()));
    }

    @Override
    public Pose2d getPoseVelocity() {
        return new Pose2d(
                pinpoint.getVelX(DistanceUnit.INCH),
                pinpoint.getVelY(DistanceUnit.INCH),
                pinpoint.getHeadingVelocity(AngleUnit.RADIANS.getUnnormalized())
        );
    }

    @Override
    public void update() {
    }
}
