package org.firstinspires.ftc.teamcode.parts.drive;

import org.firstinspires.ftc.teamcode.base.Robot;
import org.firstinspires.ftc.teamcode.base.part.RobotPartSettings;
import org.firstinspires.ftc.teamcode.other.Utils;
import org.firstinspires.ftc.teamcode.other.input.InputSupplier;

public class DriveSettings extends RobotPartSettings {
    ////////////
    //settings//
    ////////////
    //input
    Utils.GamepadNum gamepadNum = Utils.GamepadNum.ONE;
        //drive
        InputSupplier driveXSupplier = new InputSupplier(gamepad -> (gamepad.left_stick_x), gamepadNum);
        InputSupplier driveYSupplier = new InputSupplier(gamepad -> (-gamepad.left_stick_y), gamepadNum);
        InputSupplier driveRSupplier = new InputSupplier(gamepad -> (gamepad.right_stick_x), gamepadNum);
        InputSupplier driveStopSupplier = new InputSupplier(gamepad -> (gamepad.x), gamepadNum);
        InputSupplier driveSpeedSupplier = new InputSupplier(gamepad -> ((gamepad.left_bumper) ? 0.25 : (gamepad.right_bumper) ? 1 : 0.6), Utils.GamepadNum.ONE);

    //other
    public DriveMode driveMode = DriveMode.MECANUM;
    boolean useSmoothing = false;
    double[] smoothingValues = new double[]{.1,.1,.1};
    public double speedMultiplier = 1;


    @Override
    public void onInit(Robot robot) {
        driveXSupplier.init(robot);
        driveYSupplier.init(robot);
        driveRSupplier.init(robot);
        driveStopSupplier.init(robot);
    }

    public enum DriveMode{
        TANK,
        MECANUM,
        OMNI
    }
}
