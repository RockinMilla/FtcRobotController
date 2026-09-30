package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name="Single Servo Test", group="Linear OpMode")
public class SingleServoTest extends LinearOpMode {
    private static final double LUMBERJACK_POWER = 1.0;

    @Override
    public void runOpMode() {
        LinearOpMode o = this;
        RockinBotServoTest r = new RockinBotServoTest(o);

        telemetry.addData("Lumberjack Test Ready", "press PLAY");
        telemetry.addData("Controls", "Right bumper: up, left bumper: down");
        RobotLog.vv("Rockin' Robots", "Lumberjack Test Ready");
        telemetry.update();
        waitForStart();
        r.start();

        ElapsedTime telemetryTimer = new ElapsedTime();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                r.setLumberjackPower(LUMBERJACK_POWER);
            } else if (gamepad1.left_bumper) {
                r.setLumberjackPower(-LUMBERJACK_POWER);
            } else {
                r.setLumberjackPower(0);
            }

            if (telemetryTimer.seconds() >= 0.5) {
                r.printDataOnScreen();
                telemetryTimer.reset();
            }
        }
    }
}
