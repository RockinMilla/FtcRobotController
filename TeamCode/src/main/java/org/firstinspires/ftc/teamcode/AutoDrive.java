package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="Auto Drive", group="Robot")
public class AutoDrive extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o);

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "08/05/2025, 2:25 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        r.getPinpointPosition();
        r.driveToPos(0, 500, 0);
        RobotLog.vv("IDK", "taco");
        sleep(500);
        r.driveToPos(0, 500, 90);
        RobotLog.vv("IDK", "cat");
        sleep(500);
        r.driveToPos(500, 500, 90);
        RobotLog.vv("IDK", "goat");
        sleep(500);
        r.driveToPos(500, 200, 90);
        sleep(500);
        r.driveToPos(0, 200, 90);
        sleep(500);
        r.driveToPos(0, 200, 0);
        sleep(500);
        r.driveToPos(0, 0, 0);
        sleep(500);
        RobotLog.vv("Rockin' Robots", "Test Done");
        telemetry.addData("Autonomous lasted", runtime.toString());
        sleep(4 * 1000);
    }
}