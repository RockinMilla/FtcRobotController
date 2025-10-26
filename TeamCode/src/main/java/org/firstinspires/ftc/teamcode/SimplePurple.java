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

        telemetry.addData("This code was last updated", "10/26/2025, 2:32 pm"); // Todo: Update this date when the code is updated

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        r.getPinpointPosition();
        r.launcherPower(0.65);
        r.driveToPos(0, -500, 0);
        sleep(4000);
        r.lifterPower(-1.0);
        r.lifterPower(0);
        r.launcherPower(0.65);
        sleep(5000);
        r.lifterPower(-1.0);
        RobotLog.vv("Rockin' Robots", "Test Done");
        sleep(4 * 1000);
    }
}