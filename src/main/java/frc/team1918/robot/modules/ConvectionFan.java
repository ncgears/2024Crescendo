
package frc.team1918.robot.modules;
//Talon SRX/Talon FX

import edu.wpi.first.wpilibj.Relay;

//Team 1918
import frc.team1918.robot.Constants;
import frc.team1918.robot.Dashboard;

public class ConvectionFan {
    private Relay m_relay;
    public boolean enabled = false;

 	/**
	 * 1918 HotPlate Module v2023.1 - This spatula module uses a TalonSRX with 775, 550, or Bag motor on a Versa Planetary to serve scoring pieces
     * The module uses a Versa-Planetary Encoder Stage for positioning data.
     * There is a high limit and low limit switch, connected to the Talon to limit the mechanical travel
	 * @param name This is the name of this spatula module (ie. "HotPlate")
     * @param moduleConstants This is a HotPlateConstants object containing the data for this module
	 */
    public ConvectionFan(){
        m_relay = new Relay(Constants.Stove.ConvectionFan.kRelayID);

        /* Stop the relay */
        enabled=false;
        m_relay.set(Relay.Value.kOff);
    }

    /**
     *  Starts the convection fan
     */
    public void setConvectionFan(boolean enable) {
        enabled=enable;
        m_relay.set((enabled)?Relay.Value.kForward:Relay.Value.kOff);
    }

    /**
     * This function is used to output data to the dashboard for debugging the module, typically done in the periodic method.
     */
    public void updateDashboard() {
        Dashboard.ConvectionFan.setEnabled(enabled);
    }

}