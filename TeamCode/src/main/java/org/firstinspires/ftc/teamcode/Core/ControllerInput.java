package org.firstinspires.ftc.teamcode.Core;


import com.qualcomm.robotcore.hardware.Gamepad;


public class ControllerInput {
    static class Settings {
        boolean movementStick = true;
        // Left = True | Right = False
        String rotationAxis = "rx";
        double strafeCorrection = 1.1;
    } Settings settings;

    // Left stick
    double lx;
    double ly;
    // Right stick
    double rx;
    double ry;
    // Whatever stick movement is assigned to
    public double movement_x;
    public double movement_y;

    // Rotation
    public double rotation;

    public void refresh(Gamepad gamepad)
    {
        // Stick inputs
        lx =  gamepad.left_stick_x;
        ly = -gamepad.left_stick_y;
        rx =  gamepad.right_stick_x;
        ry = -gamepad.right_stick_y;

        // Movement input (facilitates stick selection within settings)
        movement_x = settings.movementStick ? lx : rx;
        movement_y = settings.movementStick ? ly : ry;

        movement_x *= settings.strafeCorrection;

        // Rotation
        if (settings.rotationAxis.equals("lx")) { rotation = lx; }
        if (settings.rotationAxis.equals("rx")) { rotation = rx; }
    }

    public void mainloop(Gamepad g)
    {
        refresh(g);
    }
}
