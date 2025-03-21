// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging.loggedObjects;


import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.lib.util.logging.LoggedContainer;
import frc.lib.util.logging.Logger.LoggingLevel;
import frc.lib.util.logging.loggedPrimitives.LoggedDoubleArray;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class LoggedSweveModules extends LoggedObject<Swerve> {

    private SwerveDriveState currentState = new SwerveDriveState();

    public void updateState(SwerveDriveState newState) {
        currentState = newState;
    }

    public LoggedSweveModules(String name, LoggedContainer subsystemLogger, Swerve object, LoggingLevel logType,
            String tabName) {
        super(name, subsystemLogger, object, logType, tabName);
    }

    public LoggedSweveModules(String name, LoggedContainer subsystemLogger, Swerve object, LoggingLevel logType,
            Boolean SeparateTab) {
        super(name, subsystemLogger, object, logType, SeparateTab);
    }

    public LoggedSweveModules(String name, LoggedContainer subsystemLogger, Swerve object, LoggingLevel logType) {
        super(name, subsystemLogger, object, logType);
    }


    public double driveRotsToMeters(double rots) {
        return rots * 2 * Math.PI * TunerConstants.kWheelRadius.in(Inches);
    }

    @Override
    protected void initializeShuffleboard() {


        ShuffleboardLayout layout0 = getTab().getLayout("Module:0", BuiltInLayouts.kList).withSize(2, 3);
        addDoubleToShuffleboard("CanCoder", () -> object.swerve.getModule(0).getEncoder().getPosition().getValueAsDouble(), layout0);
        addDoubleToShuffleboard("Angle", () -> currentState.ModuleStates[0].angle.getRotations(), layout0);
        addDoubleToShuffleboard("Speed", () -> currentState.ModuleStates[0].speedMetersPerSecond, layout0);
        addDoubleToShuffleboard("TotalDistance", () -> driveRotsToMeters(object.swerve.getModule(0).getDriveMotor().getPosition().getValueAsDouble()), layout0);
        addDoubleToShuffleboard("TotalRotations", () -> object.swerve.getModule(0).getDriveMotor().getPosition().getValueAsDouble(), layout0);

        ShuffleboardLayout layout1 = getTab().getLayout("Module:1", BuiltInLayouts.kList).withSize(2, 3);
        addDoubleToShuffleboard("CanCoder", () -> object.swerve.getModule(1).getEncoder().getPosition().getValueAsDouble(), layout1);
        addDoubleToShuffleboard("Angle", () -> currentState.ModuleStates[1].angle.getRotations(), layout1);
        addDoubleToShuffleboard("Speed", () -> currentState.ModuleStates[1].speedMetersPerSecond, layout1);
        addDoubleToShuffleboard("TotalDistance", () -> driveRotsToMeters(object.swerve.getModule(1).getDriveMotor().getPosition().getValueAsDouble()), layout1);
        addDoubleToShuffleboard("TotalRotations", () -> object.swerve.getModule(1).getDriveMotor().getPosition().getValueAsDouble(), layout1);

        ShuffleboardLayout layout2 = getTab().getLayout("Module:2", BuiltInLayouts.kList).withSize(2, 3);
        addDoubleToShuffleboard("CanCoder", () -> object.swerve.getModule(2).getEncoder().getPosition().getValueAsDouble(), layout2);
        addDoubleToShuffleboard("Angle", () -> currentState.ModuleStates[2].angle.getRotations(), layout2);
        addDoubleToShuffleboard("Speed", () -> currentState.ModuleStates[2].speedMetersPerSecond, layout2);
        addDoubleToShuffleboard("TotalDistance", () -> driveRotsToMeters(object.swerve.getModule(2).getDriveMotor().getPosition().getValueAsDouble()), layout2);
        addDoubleToShuffleboard("TotalRotations", () -> object.swerve.getModule(2).getDriveMotor().getPosition().getValueAsDouble(), layout2);

        ShuffleboardLayout layout3 = getTab().getLayout("Module:3", BuiltInLayouts.kList).withSize(2, 3);
        addDoubleToShuffleboard("CanCoder", () -> object.swerve.getModule(3).getEncoder().getPosition().getValueAsDouble(), layout3);
        addDoubleToShuffleboard("Angle", () -> currentState.ModuleStates[3].angle.getRotations(), layout3);
        addDoubleToShuffleboard("Speed", () -> currentState.ModuleStates[3].speedMetersPerSecond, layout3);
        addDoubleToShuffleboard("TotalDistance", () -> driveRotsToMeters(object.swerve.getModule(3).getDriveMotor().getPosition().getValueAsDouble()), layout3);
        addDoubleToShuffleboard("TotalRotations", () -> object.swerve.getModule(3).getDriveMotor().getPosition().getValueAsDouble(), layout3);


        // TODO Test both of these and remove one of them

        // FL, FR, BL, BR
        // ShuffleboardLayout trueStatesLayout = getTab().getLayout("TrueModuleStates", BuiltInLayouts.kList).withSize(2,
        //         3);
        // addDoubleArrayToShuffleboard(name, () -> new double[] {
        //         currentState.ModuleStates[0].angle.getDegrees(), currentState.ModuleStates[0].speedMetersPerSecond,
        //         currentState.ModuleStates[1].angle.getDegrees(), currentState.ModuleStates[1].speedMetersPerSecond,
        //         currentState.ModuleStates[2].angle.getDegrees(), currentState.ModuleStates[2].speedMetersPerSecond,
        //         currentState.ModuleStates[3].angle.getDegrees(), currentState.ModuleStates[3].speedMetersPerSecond

        // }, trueStatesLayout);

        // ShuffleboardLayout desiredStatesLayout = getTab().getLayout("DesiredModuleStates", BuiltInLayouts.kList)
        //         .withSize(2, 3);
        // addDoubleArrayToShuffleboard(name, () -> new double[] {
        //         currentState.ModuleTargets[0].angle.getDegrees(), currentState.ModuleTargets[0].speedMetersPerSecond,
        //         currentState.ModuleTargets[1].angle.getDegrees(), currentState.ModuleTargets[1].speedMetersPerSecond,
        //         currentState.ModuleTargets[2].angle.getDegrees(), currentState.ModuleTargets[2].speedMetersPerSecond,
        //         currentState.ModuleTargets[3].angle.getDegrees(), currentState.ModuleTargets[3].speedMetersPerSecond
        // }, desiredStatesLayout);



        // FL, FR, BL, BR
        add(new LoggedDoubleArray(this, () -> new double[] {
                currentState.ModuleStates[0].angle.getDegrees(), currentState.ModuleStates[0].speedMetersPerSecond,
                currentState.ModuleStates[1].angle.getDegrees(), currentState.ModuleStates[1].speedMetersPerSecond,
                currentState.ModuleStates[2].angle.getDegrees(), currentState.ModuleStates[2].speedMetersPerSecond,
                currentState.ModuleStates[3].angle.getDegrees(), currentState.ModuleStates[3].speedMetersPerSecond

        }, NetworkTableInstance.getDefault().getTable(getPrefix()).getSubTable(name).getDoubleArrayTopic("TrueStates")
                .getGenericEntry()));

        add(new LoggedDoubleArray(this, () -> new double[] {
                currentState.ModuleTargets[0].angle.getDegrees(), currentState.ModuleTargets[0].speedMetersPerSecond,
                currentState.ModuleTargets[1].angle.getDegrees(), currentState.ModuleTargets[1].speedMetersPerSecond,
                currentState.ModuleTargets[2].angle.getDegrees(), currentState.ModuleTargets[2].speedMetersPerSecond,
                currentState.ModuleTargets[3].angle.getDegrees(), currentState.ModuleTargets[3].speedMetersPerSecond

        }, NetworkTableInstance.getDefault().getTable(getPrefix()).getSubTable(name)
                .getDoubleArrayTopic("DesiredStates").getGenericEntry()));

    }

    @Override
    protected void initializeDataLog() {
        addDoubleToOnboardLog("Module:0/CanCoder", () -> object.swerve.getModule(0).getEncoder().getPosition().getValueAsDouble());
        addDoubleToOnboardLog("Module:0/TotalDistance", () -> driveRotsToMeters(object.swerve.getModule(0).getDriveMotor().getPosition().getValueAsDouble()));

        addDoubleToOnboardLog("Module:1/CanCoder", () -> object.swerve.getModule(1).getEncoder().getPosition().getValueAsDouble());
        addDoubleToOnboardLog("Module:1/TotalDistance", () -> driveRotsToMeters(object.swerve.getModule(1).getDriveMotor().getPosition().getValueAsDouble()));

        addDoubleToOnboardLog("Module:2/CanCoder", () -> object.swerve.getModule(2).getEncoder().getPosition().getValueAsDouble());
        addDoubleToOnboardLog("Module:2/TotalDistance", () -> driveRotsToMeters(object.swerve.getModule(2).getDriveMotor().getPosition().getValueAsDouble()));

        addDoubleToOnboardLog("Module:3/CanCoder", () -> object.swerve.getModule(3).getEncoder().getPosition().getValueAsDouble());
        addDoubleToOnboardLog("Module:3/TotalDistance", () -> driveRotsToMeters(object.swerve.getModule(3).getDriveMotor().getPosition().getValueAsDouble()));

        addDoubleArrayToOnboardLog("TrueStates", () -> new double[] {
                currentState.ModuleStates[0].angle.getDegrees(), currentState.ModuleStates[0].speedMetersPerSecond,
                currentState.ModuleStates[1].angle.getDegrees(), currentState.ModuleStates[1].speedMetersPerSecond,
                currentState.ModuleStates[2].angle.getDegrees(), currentState.ModuleStates[2].speedMetersPerSecond,
                currentState.ModuleStates[3].angle.getDegrees(), currentState.ModuleStates[3].speedMetersPerSecond
        });

        addDoubleArrayToOnboardLog("DesiredStates", () -> new double[] {
                currentState.ModuleTargets[0].angle.getDegrees(), currentState.ModuleTargets[0].speedMetersPerSecond,
                currentState.ModuleTargets[1].angle.getDegrees(), currentState.ModuleTargets[1].speedMetersPerSecond,
                currentState.ModuleTargets[2].angle.getDegrees(), currentState.ModuleTargets[2].speedMetersPerSecond,
                currentState.ModuleTargets[3].angle.getDegrees(), currentState.ModuleTargets[3].speedMetersPerSecond
        });
    }

}
