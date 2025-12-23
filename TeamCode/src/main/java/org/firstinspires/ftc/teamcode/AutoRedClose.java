package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Close", group="Robot")
public class AutoRedClose extends LinearOpMode {
    int launcherVel = 800;
    int lifterVel = 1000;

    @Override
    public void runOpMode() {
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o,"Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "12/18/2025, 2:45 pm"); // Todo: Update this date when the code is updated

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        r.getPinpointPosition();
        r.launcherVelocity(launcherVel);

        shootBalls(r); // Shoot preloaded balls

        r.driveToPos(-200, -800, 37); // park
        r.launcherVelocity(0);
        r.intakePower(0);
    }

    private void shootBalls(RockinBot r)
    {
        r.intakePower(0.2);
        r.driveToPos(100, -700, 45);
        r.intakePower(0.5);
        r.waitForLaunchers(launcherVel);
        r.lifterVelocity(lifterVel, 2);
        r.turnLifterToDegrees(-190); // Reset lifter
        r.waitForLifter();
    }
}