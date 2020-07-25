package us.ihmc.yoVariables.parameters;

public class ParameterData
{
   private final String value;
   private final double min;
   private final double max;

   public ParameterData(String value, String min, String max)
   {
      this.value = value;
      if (min != null && max != null)
      {
         this.min = Double.parseDouble(min);
         this.max = Double.parseDouble(max);
      }
      else
      {
         this.min = 0.0;
         this.max = 1.0;
      }
   }

   public ParameterData(String value)
   {
      this.value = value;
      min = 0.0;
      max = 1.0;
   }

   public void setParameterFromThis(YoParameter parameter)
   {
      parameter.load(value);
      parameter.setParameterBounds(min, max);
   }
}
