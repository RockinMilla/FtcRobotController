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

        telemetry.addData("This code was last updated", "11/9/2025, 2:40pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        r.launcherPower(0.38);
        r.intakePower(0.4);
        sleep(500); // Wait for motors to get up to speed
        r.driveBack(1000); // Drive to shooting position
        sleep(2000);
        r.lifterPower(0.3);
        sleep(1000); // Shoot
        r.lifterPower(0);
        r.intakePower(0.7);
        sleep(1000); // Pause
        r.lifterPower(0.4);
        sleep(1000); // Shoot
        r.launcherPower(0);
        r.lifterPower(0);
        r.intakePower(0); // Set all motors to 0
        r.driveRight(500); // Get off the line
        sleep(2000);
        RobotLog.vv("Rockin' Robots", "Test Done");
    }
}