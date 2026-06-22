package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.localizer.Localizer;
import org.firstinspires.ftc.teamcode.localizer.pinpoint.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.util.math.units.Pose2d;

public class TestOpMode extends LinearOpMode {
    private final Localizer localizer;

    public TestOpMode() {
         localizer = new PinpointLocalizer(hardwareMap);

         localizer.setPose(new Pose2d(0, 0, 0));
    }
    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Pose", localizer.getPose().toString());
            telemetry.addData("Velocity", localizer.getVelocity().toString());
        }
    }
}
