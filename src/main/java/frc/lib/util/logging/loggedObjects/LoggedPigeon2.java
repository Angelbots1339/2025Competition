// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging.loggedObjects;


import com.ctre.phoenix6.hardware.Pigeon2;


import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.lib.util.logging.LoggedContainer;
import frc.lib.util.logging.Logger.LoggingLevel;


/** Add your docs here. */
public class LoggedPigeon2 extends LoggedObject<Pigeon2> {

    public LoggedPigeon2(String name, LoggedContainer subsystemLogger, Pigeon2 object, LoggingLevel logType, String tabName) {
        super(name, subsystemLogger, object, logType, tabName);
    }

    public LoggedPigeon2(String name, LoggedContainer subsystemLogger, Pigeon2 object, LoggingLevel logType,
            Boolean SeptateTab) {
        super(name, subsystemLogger, object, logType, SeptateTab);
    }

    public LoggedPigeon2(String name, LoggedContainer subsystemLogger, Pigeon2 object, LoggingLevel logType) {
        super(name, subsystemLogger, object, logType);
    }

    @Override
    protected void initializeShuffleboard() {
        ShuffleboardLayout layout = getTab().getLayout(name, BuiltInLayouts.kList).withSize(1, 3);
        addDoubleToShuffleboard("Yaw", () -> object.getYaw().getValueAsDouble(), layout);
        addDoubleToShuffleboard("Pitch", () -> object.getPitch().getValueAsDouble(), layout);

        layout.addDoubleArray(name, () -> {
            short[] accel = new short[3];
            accel[0] = (short) object.getAccelerationX().getValueAsDouble();
            accel[1] = (short) object.getAccelerationY().getValueAsDouble();
            accel[2] = (short) object.getAccelerationZ().getValueAsDouble();
            return new double[]{accel[0],accel[1],accel[2]};
        }).withWidget(BuiltInWidgets.k3AxisAccelerometer);

    }

    @Override
    protected void initializeDataLog() {
        addDoubleToOnboardLog("Yaw", () -> object.getYaw().getValueAsDouble());
        addDoubleToOnboardLog("Pitch", () -> object.getPitch().getValueAsDouble());
        addDoubleToOnboardLog("Roll", () -> object.getRoll().getValueAsDouble());

        addDoubleArrayToOnboardLog("Accelerometer [x,y,z]", () -> {
            short[] accel = new short[3];
            accel[0] = (short) object.getAccelerationX().getValueAsDouble();
			accel[1] = (short) object.getAccelerationY().getValueAsDouble();
			accel[2] = (short) object.getAccelerationZ().getValueAsDouble();
			return new double[] { accel[0], accel[1], accel[2] };
        });

    }

}
