/*----------------------------------------------------------------------------*/
/* Copyright (c) 2021 NC Gears Team 1918. All Rights Reserved.                */
/* Open Source Software - may be modified and shared by FRC teams.            */
/* No attribution is necessary to use this code                               */
/*----------------------------------------------------------------------------*/
package frc.team1918.robot.utils;

import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;

@Deprecated(since = "2023")
public class OrPOVButton extends Button {
    private final POVButton button1;
    private final POVButton button2;
    private final POVButton button3;
        
    /**
     * This function returns a POVButton object that is a combination of three other POVButtons.  The get function returns true if any of the POVButtons is true.
     * @param buttonOne The first POVButton object
     * @param buttonTwo The second POVButton object
     * @param buttonThree The third POVButton object
     */
    public OrPOVButton(POVButton buttonOne, POVButton buttonTwo, POVButton buttonThree) {
        button1 = buttonOne;
        button2 = buttonTwo;
        button3 = buttonThree;
    }

    @Override
    public boolean getAsBoolean() {
        return button1.getAsBoolean() || button2.getAsBoolean() || button3.getAsBoolean();
    }
}