package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="Simple Purple", group="Robot")
public class SimplePurple extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o,"Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");

        telemetry.addData("This code was last updated", "10/31/2025, 3:26" + " pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        r.lifterPower(0.05);
        r.launcherPower(0.55);
        r.intakePower(-0.1);
        sleep(500);
        r.driveBack(1000);
        sleep(2000);
        //r.intakePower(-0.7);
        r.lifterPower(-0.7);
        sleep(200);
        r.intakePower(-0.7);
        r.lifterPower(0.05);
        sleep(1000);
        for(int i = 1; i<4; i++) {
            r.lifterPower(-0.6);
            sleep(500);
            r.lifterPower(0.05);
            sleep(2000);
        }
        r.launcherPower(0);
        r.lifterPower(0);
        r.intakePower(0);
        r.driveRight(500);
        sleep(2000);
        RobotLog.vv("Rockin' Robots", "Test Done");
    }
}