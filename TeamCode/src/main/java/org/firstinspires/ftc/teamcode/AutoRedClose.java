package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="Red Close", group="Robot")
public class AutoRedClose extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o,"Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");

        telemetry.addData("This code was last updated", "11/30/2025, 2:45 pm"); // Todo: Update this date when the code is updated

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        r.getPinpointPosition();
        //r.intakePower(0.5);
        r.launcherVelocity(800);

        shootBalls(r); // Shoot preloaded balls

        // Pick up 1st set of balls
        r.driveToPos(720, -900, 0, 15, 3, 4);
        r.turnLifterByDegrees(270, 300);
        r.intakePower(0.9);
        r.driveToPos(720, 0, 0, 15, 3, 3);
        r.turnLifterByDegrees(-10, 700);
        sleep(700);

        shootBalls(r);

        // Pick up 2nd set of balls
        r.driveToPos(1350, -900, 0, 15, 3, 4);
        r.turnLifterByDegrees(270, 250);
        r.intakePower(0.9);
        r.driveToPos(1350, 200, 0, 15, 3, 3);
        r.turnLifterByDegrees(-10, 700);
        sleep(300);

        r.driveToPos(1400, -300, 0); // move to avoid gate

        r.turnLifterByDegrees(-10, 700);
        shootBalls(r);
        r.driveToPos(-200, -800, 37); // park
        r.launcherVelocity(0);
        r.intakePower(0);

        //RobotLog.vv("Rockin' Robots", "Test Done");
    }

    private void shootBalls(RockinBot r)
    {
        r.intakePower(-0.05);
        r.driveToPos(100, -700, 37);
        r.intakePower(0.5);
        sleep(500);
        r.waitForLaunchers(800);
        r.turnLifterByDegrees(90, 1400);
        r.launcherVelocity(800);
        r.waitForLifter();
        r.waitForLaunchers(800);
        r.turnLifterByDegrees(90, 1350);
        r.waitForLifter();
        r.waitForLaunchers(800);
        r.turnLifterByDegrees(360, 1200);
        r.waitForLifter();
    }
}