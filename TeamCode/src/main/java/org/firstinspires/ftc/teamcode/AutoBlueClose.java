package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Close", group="Robot")
public class AutoBlueClose extends LinearOpMode {

    int launcherVel = 800;
    int lifterVel = 1000;

    @Override
    public void runOpMode() {

        LinearOpMode o = this;
        RockinBot r = new RockinBot(o, "Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "12/17/2025, 11:15 am"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        r.getPinpointPosition();
        r.launcherVelocity(launcherVel);

        shootBalls(r); // Shoot preloaded balls

        r.driveToPos(100, -800, -35); // park
        r.launcherVelocity(0);
        r.intakePower(0);
    }

    private void shootBalls(RockinBot r)
    {
        r.intakePower(0.2);
        r.driveToPos(0, -700, -32);
        r.intakePower(0.5);
        r.waitForLaunchers(launcherVel);
        r.turnLifterToDegrees(360, lifterVel); // Shoot all three balls
        r.waitForLifter();
        r.turnLifterToDegrees(-190); // Reset lifter
    }
}