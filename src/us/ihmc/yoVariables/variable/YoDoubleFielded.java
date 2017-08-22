package us.ihmc.yoVariables.variable;

import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.lang.reflect.Method;

public class YoDoubleFielded extends YoDouble
{
   protected Method getValue;

   protected Object fromObject;

   public YoDoubleFielded(String withName, YoVariableRegistry forRegistry, Object fromObject, Method getValue)
   {
      super(withName, forRegistry);

      this.fromObject = fromObject;

      this.getValue = getValue;
   }

   @Override
   public double getDoubleValue()
   {
      double value = Double.NaN;

      try
      {
         value = (double) this.getValue.invoke(this.fromObject);
      }
      catch (Exception ignored)
      {
         // do nothing
      }

      return value;
   }

   @Override
   public double getValueAsDouble() {
      return getDoubleValue();
   }
}
