package us.ihmc.yoVariables.listener;

import us.ihmc.yoVariables.variable.YoVariable;

/**
 * @author Jerry Pratt
 */

public interface VariableChangedListener
{
   public void variableChanged(YoVariable<?> v);
}
