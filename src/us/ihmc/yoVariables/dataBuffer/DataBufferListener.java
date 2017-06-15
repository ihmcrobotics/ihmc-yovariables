package us.ihmc.yoVariables.dataBuffer;

import us.ihmc.yoVariables.YoVariableHolder;
import us.ihmc.yoVariables.variable.DoubleYoVariable;


public interface DataBufferListener
{
   public DoubleYoVariable[] getVariablesOfInterest(YoVariableHolder yoVariableHolder);

   public void dataBufferUpdate(double[] values);
}
