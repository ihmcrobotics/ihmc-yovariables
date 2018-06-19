package us.ihmc.yoVariables.parameters;

public class ParameterData
{
   private final String value;
   private final double min;
   private final double max;

   public ParameterData(String value, String min, String max)
   {
      this.value = value;
      this.min = Double.parseDouble(min);
      this.max = Double.parseDouble(max);
   }

   public ParameterData(String value)
   {
      this.value = value;
      this.min = 0.0;
      this.max = 1.0;
   }

   public void setParameterFromThis(YoParameter<?> parameter)
   {
      parameter.load(value);
      parameter.setSuggestedRange(min, max);
   }
}
