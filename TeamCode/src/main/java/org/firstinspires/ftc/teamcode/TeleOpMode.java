package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.localizer.PoseEstimator;
import org.firstinspires.ftc.teamcode.localizer.pinpoint.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.util.math.units.Vector2d;

@TeleOp
public class TeleOpMode extends LinearOpMode {
    private MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap, new PoseEstimator(new PinpointLocalizer(hardwareMap)));

    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            drivetrain.setToFieldRelativePercent(
                    new Vector2d(
                            gamepad1.left_stick_x,
                            gamepad1.left_stick_y
                    ),
                    gamepad1.right_stick_x
            );

            drivetrain.update();
        }
    }
}
