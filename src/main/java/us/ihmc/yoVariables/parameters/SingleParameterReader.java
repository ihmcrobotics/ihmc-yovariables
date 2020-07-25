package us.ihmc.yoVariables.parameters;

/**
 * Provides tools for initializing a single parameter.
 * <p>
 * The main usecase for this class is for deserialization of parameters.
 * </p>
 */
public class SingleParameterReader
{
   /**
    * Initializes the given parameter from the given double value.
    * 
    * @param parameter   the parameter to be initialized.
    * @param doubleValue the initial value to use on the parameter.
    * @param loadStatus  indicator of the origin of the given double value.
    * @throws RuntimeException if {@code loadedStatus == ParameterLoadStatus.UNLOADED}.
    */
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
