package util;

/**
 *
 * Created by Someone.
 */


//TODO: Perhaps change this to include a subsystem state enum that's passed in

public class Toggler
{
    private boolean pressedDec = false;
    private boolean pressedInc = false;
    private boolean limited = false;
    private int state = 0;
    private int numberOfStates = 0;

    /**
     * Sets up the toggler object.
     * @param stateNum The number of states the util.Toggler will have
     * @param initState The starting state of the util.Toggler
     */
    public Toggler(int stateNum, int initState)
    {
        numberOfStates = stateNum;
        limited = true;
        state = (initState < stateNum) && (initState > -1) ? initState: 0;
    }

    /**
     * Sets up the util.Toggler object
     * @param stateNum The number of states the util.Toggler will have
     */
    public Toggler(int stateNum)
    {
        numberOfStates = stateNum;
    }

    /**
     * The current state that the toggler is in
     * @return Current state value, as an int
     */
    public int currentState()
    {
        return state;
    }

    public int howManyStates()
    {
        return numberOfStates;
    }

    /**
     * Increments or decrements the state of the util.Toggler
     *
     * This boolean trigger can have an expression with && or || for further functionality
     *
     * @param inc Boolean which will increment the util.Toggler. Keep in mind since this is a util.Toggler, holding will do nothing.
     * @param dec Boolean which will increment the util.Toggler. Keep in mind since this is a util.Toggler, holding will do nothing.
     */
    public void changeState(boolean inc, boolean dec)
    {
        if(inc)
        {
            if(!pressedInc)
            {
                if(limited)
                {
                    state = ((state + 1) != numberOfStates)
                            ? (state + 1) % numberOfStates
                            :state;
                }
                else {
                    state = (state + 1) % numberOfStates;
                }
            }
            pressedInc = true;
        }
        else {
            pressedInc = false;
        }

        if(dec)
        {
            if(!pressedDec)
            {
                if(limited)
                {
                    state = ((state - 1) != -1)
                            ? ((state - 1) % numberOfStates+numberOfStates)%numberOfStates
                            :state;
                }
                else {
                    state = ((state - 1) % numberOfStates+numberOfStates)%numberOfStates;
                }
            }
            pressedDec = true;
        }
        else
        {
            pressedDec = false;
        }
    }

    /**
     * Increments or decrements the state of the util.Toggler
     *
     * This boolean trigger can have an expression with && or || for further functionality
     *
     * When used with a util.Toggler with only two states, this method can be used to create a simple on/off util.Toggler.
     *
     * @param inc Boolean which will increment the util.Toggler. Keep in mind since this is a util.Toggler, holding will do nothing.
     */
    public void changeState(boolean inc)
    {
        if(inc)
        {
            if(!pressedInc)
            {
                if(limited)
                {
                    state = ((state + 1) != numberOfStates)
                            ? (state + 1) % numberOfStates
                            :state;
                }
                else {
                    state = (state + 1) % numberOfStates;
                }
            }
            pressedInc = true;
        }
        else
        {
            pressedInc = false;
        }
    }

    /**
     * Directly sets the current state of the util.Toggler (not toggling)
     * @param stateL State to set the util.Toggler to. Is filtered to make sure it is
     *               not higher than the number of states or less or equal to 0
     * @return The newly set state of the util.Toggler
     */
    public int setState(int stateL)
    {
        if(stateL < numberOfStates & stateL >= 0)
        {
            state = stateL;
        }
        return state;
    }
}
