/*----------------------------------------------------------------------------*/
/* Copyright (c) 2021 NC Gears Team 1918. All Rights Reserved.                */
/* Open Source Software - may be modified and shared by FRC teams.            */
/* No attribution is necessary to use this code                               */
/*----------------------------------------------------------------------------*/
package frc.team1918.robot.utils;

import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class creates a Button object that requires 2 buttons to be pressed at the same time.
 */
@Deprecated(since = "2023")
public class AndButton extends Button {
    private final Button button1;
    private final Button button2;

    /**
     * This function returns a Button object that is a combination of two other Buttons.  The get function requires both to be true.
     * @param buttonOne The first button object
     * @param buttonTwo The second button object
     */
    public AndButton(Button buttonOne, Button buttonTwo) {
        button1 = buttonOne;
        button2 = buttonTwo;
    }

    @Override
    public boolean getAsBoolean() {
        return button1.getAsBoolean() && button2.getAsBoolean();
    }
}