package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

// @Autonomous(name="Simple Purple", group="Robot")
public class SimplePurple extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o,"Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");

        telemetry.addData("This code was last updated", "11/15/2025, 12:31pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        r.launcherVelocity(0.35);
        r.intakePower(1);
        sleep(500); // Wait for motors to get up to speed
        r.driveBack(1250); // Drive to shooting position
        sleep(1000);
        r.lifterPower(0.2); // Lift first ball
        sleep(500); // Shoot first ball
        r.lifterPower(0);
        r.launcherVelocity(0.37); // Turn up launcher speed a bit to account for the first ball slowing it down a little
        sleep(1000); // Pause to let launcher get up to speed
        r.lifterPower(-0.2); // Go backwards a bit to prepare for shooting the second ball
        sleep(400);
        r.lifterPower(0.5); // Lift second ball
        sleep(2000); // Shoot second ball
        r.launcherVelocity(0);
        r.lifterPower(0);
        r.intakePower(0); // Set all motors to 0
        r.driveRight(500); // Get off the line
        sleep(2000);
        RobotLog.vv("Rockin' Robots", "Test Done");
    }
}