package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name="Single Servo Test", group="Linear OpMode")
public class SingleServoTest extends LinearOpMode {
    private static final double POSITION_CHANGE_PER_SECOND = 0.5;

    @Override
    public void runOpMode() {
        LinearOpMode o = this;
        RockinBotServoTest r = new RockinBotServoTest(o);

        telemetry.addData("Lumberjack Test Ready", "press PLAY");
        telemetry.addData("Controls", "Hold right bumper: up, hold left bumper: down");
        RobotLog.vv("Rockin' Robots", "Lumberjack Test Ready");
        telemetry.update();
        waitForStart();
        r.start();

        ElapsedTime telemetryTimer = new ElapsedTime();
        ElapsedTime movementTimer = new ElapsedTime();
        boolean wasMoving = false;

        while (opModeIsActive()) {
            double positionChange = POSITION_CHANGE_PER_SECOND * movementTimer.seconds();
            movementTimer.reset();
            boolean moving = gamepad1.right_bumper ^ gamepad1.left_bumper;

            if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                r.adjustLumberjackPosition(positionChange);
            } else if (gamepad1.left_bumper && !gamepad1.right_bumper) {
                r.adjustLumberjackPosition(-positionChange);
            } else if (wasMoving) {
                r.holdLumberjackPosition();
            }
            wasMoving = moving;

            if (telemetryTimer.seconds() >= 0.1) {
                r.printDataOnScreen(gamepad1.right_bumper, gamepad1.left_bumper);
                telemetryTimer.reset();
            }

            idle();
        }
    }
}
