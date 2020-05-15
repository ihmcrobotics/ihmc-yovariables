package us.ihmc.yoVariables.dataBuffer;

import us.ihmc.yoVariables.variable.YoDouble;

public interface DataBufferListener
{
   public YoDouble[] getVariablesOfInterest(YoVariableHolder yoVariableHolder);

   public void dataBufferUpdate(double[] values);
}
