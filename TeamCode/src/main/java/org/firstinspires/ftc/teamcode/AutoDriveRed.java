package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name="Red", group="Robot")
public class AutoDriveRed extends LinearOpMode {
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
        r.intakePower(1.0);
        r.launcherVelocity(0.34);

        shootBalls(r);

        r.driveToPos(740, -1000, 0);
        r.lifterPower(0.2);
        r.driveToPos(740, -50, 0);
        r.lifterPower(0);
        sleep(500);

        shootBalls(r);

        r.driveToPos(1350, -800, 0);
        r.lifterPower(0.2);
        r.driveToPos(1350, 100, 0);
        r.lifterPower(0);

        r.driveToPos(1350, -300, 0); // move to avoid gate

        shootBalls(r);

        r.launcherVelocity(0);
        r.intakePower(0);

        RobotLog.vv("Rockin' Robots", "Test Done");
    }
    private void shootBalls(RockinBot r)
    {
        r.driveToPos(100, -700, 40);
        r.lifterPower(0.6);
        sleep(300);
        r.lifterPower(0);
        sleep(500);
        r.lifterPower(0.6);
        sleep(300);
        r.lifterPower(0);
        sleep(500);
    }
}