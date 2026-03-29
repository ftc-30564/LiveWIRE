package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.localizer.Localizer;
import org.firstinspires.ftc.teamcode.localizer.pinpoint.PinpointConstants;
import org.firstinspires.ftc.teamcode.localizer.pinpoint.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.util.math.units.Pose2d;
import org.firstinspires.ftc.teamcode.util.math.units.Distance;

public class TestOpMode extends LinearOpMode {
    private final Localizer localizer;

    public TestOpMode() {
         localizer = new PinpointLocalizer(hardwareMap,
                new PinpointConstants(
                    "pinpoint",
                        Distance.fromInches(5),
                        Distance.fromInches(5),
                        GoBildaPinpointDriver.EncoderDirection.FORWARD,
                        GoBildaPinpointDriver.EncoderDirection.FORWARD,
                        GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
                ));

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
