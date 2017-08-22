package us.ihmc.yoVariables.variable;

import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.lang.reflect.Method;

public class YoIntegerFielded extends YoInteger
{
   Method getValue;

   Object fromObject;

   public YoIntegerFielded(String withName, YoVariableRegistry forRegistry, Object fromObject, Method getValue)
   {
      super(withName, forRegistry);

      this.fromObject = fromObject;

      this.getValue = getValue;
   }

   @Override
   public int getIntegerValue()
   {
      int value = Integer.MIN_VALUE;

      try
      {
         value = (int) this.getValue.invoke(this.fromObject);
      }
      catch (Exception ignored)
      {
         // do nothing
      }

      return value;
   }

   @Override
   public double getValueAsDouble() {
      return (double) getIntegerValue();
   }
}
