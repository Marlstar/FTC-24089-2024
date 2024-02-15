package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

// All the imports the IMU wanted:
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.Core.Vector;


public class Robot {
    public ControllerInput controllerInput;
    public HardwareMap hwMap;

    /**
     * @param h A reference to the "hardwareMap"
     */
    public void create(HardwareMap h)
    {
        hwMap = h;
    }

    public class Drive {
        public class Motors {
            // MOVEMENT
            // Drive motors
            public DcMotor frontLeftDriveMotor;
            public DcMotor frontRightDriveMotor;
            public DcMotor backLeftDriveMotor;
            public DcMotor backRightDriveMotor;



            // INTAKE
            // Spin motors (1150RPMs)
            public DcMotor leftIntakeSpinMotor;
            public DcMotor rightIntakeSpinMotor;
            // Position motors
            public Servo leftIntakeRaise;
            public Servo rightIntakeRaise;

            public void init() {
                // Initialising motors
                // MOVEMENT
                // Drive motors
                frontLeftDriveMotor = hwMap.get(DcMotor.class, "frontLeftDriveMotor");
                frontRightDriveMotor = hwMap.get(DcMotor.class, "frontRightDriveMotor");
                backLeftDriveMotor = hwMap.get(DcMotor.class, "backLeftDriveMotor");
                backRightDriveMotor = hwMap.get(DcMotor.class, "backRightDriveMotor");
                // INTAKE
                // Spin motors (1150RPMs)
                // leftIntakeSpinMotor = hwMap.get(DcMotor.class, "leftIntakeMotor");
                /// rightIntakeSpinMotor = hwMap.get(DcMotor.class, "rightIntakeMotor");
                // Raise servos
                // leftIntakeRaise = hwMap.get(Servo.class, "leftIntakeRaise");
                // rightIntakeRaise = hwMap.get(Servo.class, "rightIntakeRaise");
            }
        } public Motors motors;

        public class IMU {
            public IMU imu;
            public double targetYaw;
            public double initalYaw;
            public double lastError = 0.0;
            public void resetIMU(){ initIMU(); }

            public void initIMU() {
                com.qualcomm.robotcore.hardware.IMU.Parameters parameters = new com.qualcomm.robotcore.hardware.IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                        )
                );

                imu = hardwareMap.get(com.qualcomm.robotcore.hardware.IMU.class, "imu");
                imu.initialize(parameters);
            }
            // This is copied from my block code last year.
            public double getYawDegrees() {
                Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                return AngleUnit.DEGREES.normalize(angles.firstAngle);
            }

            // Another wrapper function
            public void init() {
                initIMU();
            }

        } public IMU robotIMU;


        public class Movement {
            class DesiredMovements {
                public double frontLeftDrive;
                public double frontRightDrive;
                public double backLeftDrive;
                public double backRightDrive;
            } public DesiredMovements desiredMovements;

            public double calculate_PID(double kP, double kD, double currentError, double lastError) {
                // Do not change this.
                double correction = ((currentError * kP ) + (kD * (currentError - lastError))) * -0.1;
                return correction;
            }

            public double calculateRotationPower(double targetRotation, double threshold) {
                double currentRotation = robotIMU.getYawDegrees();
                double error = targetRotation - currentRotation;
                // Such that if the robot is at 15d and the targetRotation = 0d,
                // this will be equal to -15d. For now I'll use a > or < but once
                // it works, I will switch to PID.

                double correction = 0;

                if (Math.abs(error) > threshold) { correction = calculate_PID(0.2, 0.5, error, robotIMU.lastError); }

                double speed = 1.0;

                if (correction < -1) { correction = -1;}
                if (correction > 1) { correction = 1; }

                robotIMU.lastError = error;

                // I'm adding proper smoothing later this was proof of concept
                if (Math.abs(error) > 80) { speed = 0.8;}
                if (Math.abs(error) > 100) { speed = 0.5;}
                if (Math.abs(error) > 120) { speed = 0.3;}
                if (Math.abs(error) > 160) { speed = 0.2;}

                return (correction + (Math.abs(correction) / correction) * 0.04) * speed;
            }

            void setLeftSidePower(double power) {
                desiredMovements.frontLeftDrive = power;
                desiredMovements.backLeftDrive = power;
            }

            void setRightSidePower(double power) {
                desiredMovements.frontRightDrive = power;
                desiredMovements.backRightDrive = power;
            }

            void addFlBrDiagonal(double power) {
                desiredMovements.frontLeftDrive += power;
                desiredMovements.backRightDrive += power;
            }

            void addFrBlDiagonal(double power) {
                desiredMovements.frontRightDrive += power;
                desiredMovements.backLeftDrive += power;
            }

            void driveInDirection(double degrees, double power) {
                degrees -= robotIMU.getYawDegrees();
                Vector driveVector = new Vector();
                if (degrees < -180) { degrees += 360;}
                if (degrees > 180) { degrees -= 360;}
                driveVector.fromPolar(degrees, power / 2.0);
                // ^^ gives the vector in which we need to drive (div 2.0 for now)
                setLeftSidePower(driveVector.y);
                setRightSidePower(driveVector.y);
                addFrBlDiagonal(driveVector.x * -1);
                addFlBrDiagonal(driveVector.x);
                setMotorPowers();
            }

            void stopMotors() {
                setLeftSidePower(0.0);
                setRightSidePower(0.0);
            }

            void calculateMovementTele() {
                double mx = controllerInput.movement_x;
                double my = controllerInput.movement_y;
                double controllerR = controllerInput.rotation;
                robotIMU.targetYaw += controllerR; // Assuming -1 -> 1
                if (robotIMU.targetYaw < -180) { robotIMU.targetYaw += 360; }
                if (robotIMU.targetYaw >  180) { robotIMU.targetYaw -= 360; }
                double r = calculateRotationPower(robotIMU.targetYaw, 3.0);

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(my) + Math.abs(mx) + Math.abs(r), 1);

                //If we are correcting yaw, slow down the motors so that it makes a difference
                // (If they are all 100% speed then decrease to 80% so that the motors can speed up
                // more to rotate). Learnt this last year.
                double movementMultiplier = 1.0 - Math.abs(r);

                // Marley's original code (modified to add a movement multiplier)
                desiredMovements.frontLeftDrive = ((mx + my) * movementMultiplier + r) / denominator;
                desiredMovements.frontRightDrive = ((mx - my) * movementMultiplier - r) / denominator;
                desiredMovements.backLeftDrive = ((mx - my) * movementMultiplier + r) / denominator;
                desiredMovements.backRightDrive = ((mx + my) * movementMultiplier - r) / denominator;
            }

            public void setMotorPowers()
            {
                motors.frontLeftDriveMotor.setPower(desiredMovements.frontLeftDrive);
                motors.frontRightDriveMotor.setPower(desiredMovements.frontRightDrive);
                motors.backLeftDriveMotor.setPower(desiredMovements.backLeftDrive);
                motors.backRightDriveMotor.setPower(desiredMovements.backRightDrive);
            }

            public void moveTele()
            {
                calculateMovementTele();
                setMotorPowers();
            }
        } public Movement movement;
    } public Drive drive;
}
