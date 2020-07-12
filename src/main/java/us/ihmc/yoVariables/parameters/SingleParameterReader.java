package us.ihmc.yoVariables.parameters;

public class SingleParameterReader
{
   public static void readParameter(YoParameter parameter, double doubleValue, ParameterLoadStatus loadStatus)
   {
      if (loadStatus == ParameterLoadStatus.UNLOADED)
      {
         throw new RuntimeException("Can not load parameter and set the status to unloaded.");
      }
      parameter.getVariable().setValueFromDouble(doubleValue);
      parameter.loadStatus = loadStatus;
   }
}
