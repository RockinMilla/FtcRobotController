package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Close", group="Robot")
public class AutoBlueClose extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        LinearOpMode o = this;
        RockinBot r = new RockinBot(o, "Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "12/8/2025, 11:15 am"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        r.getPinpointPosition();
        //r.intakePower(0.5);
        r.launcherVelocity(770);

        shootBalls(r); // Shoot preloaded balls

        // Pick up 1st set of balls
        r.driveToPos(-780, -900, 0, 15, 3, 3);
        r.turnLifterByDegrees(250, 600);
        r.intakePower(0.8);
        r.driveToPos(-780, 0, 0, 15, 3, 2);
        r.waitForLifter();
        sleep(700);

        shootBalls(r);

        // Pick up 2nd set of balls
        r.driveToPos(-1400, -900, 0, 15, 3, 3);
        r.turnLifterByDegrees(230, 500);
        r.intakePower(0.8);
        r.driveToPos(-1400, 200, 0, 15, 3, 2);
        r.waitForLifter();
        r.turnLifterByDegrees(-10, 700);
        sleep(300);

        r.driveToPos(-1400, -300, 0); // move to avoid gate

        r.turnLifterByDegrees(-10, 700);
        shootBalls(r);
        r.driveToPos(100, -800, -35); // park
        r.launcherVelocity(0);
        r.intakePower(0);

        //RobotLog.vv("Rockin' Robots", "Test Done");
    }

    private void shootBalls(RockinBot r)
    {
        r.intakePower(0.2);
        r.driveToPos(-150, -750, -35);
        r.intakePower(0.5);
        sleep(500);
        r.waitForLaunchers(800);
        sleep(200);
        r.turnLifterByDegrees(90, 1500);
        r.waitForLifter();
        r.turnLifterByDegrees(-10, 1000);
        r.waitForLifter();
        r.waitForLaunchers(800);
        sleep(200);
        r.turnLifterByDegrees(100, 1350);
        r.waitForLifter();
        r.waitForLaunchers(800);
        sleep(200);
        r.turnLifterByDegrees(360, 1250);
        r.waitForLifter();
    }
}