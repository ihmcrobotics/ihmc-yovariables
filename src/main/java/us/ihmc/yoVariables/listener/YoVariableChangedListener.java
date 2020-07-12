package us.ihmc.yoVariables.listener;

import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Listener on the backing value of a {@link YoVariable}.
 *
 * @author Jerry Pratt
 */
public interface YoVariableChangedListener
{
   /**
    * Called when the primitive data-type backing a YoVariable is changed
    *
    * @param source YoVariable the listener is attached to
    */
   void changed(YoVariable source);
}
