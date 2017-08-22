package us.ihmc.yoVariables.variable;

import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.lang.reflect.Method;

public class YoDoubleFieldedArray extends YoDoubleFielded {
   private int index;

   public YoDoubleFieldedArray(String withName, YoVariableRegistry forRegistry, Object fromObject, Method getValue, int index) {
      super(withName, forRegistry, fromObject, getValue);

      this.index = index;
   }

   @Override
   public double getDoubleValue() {
      double value = Double.NaN;

      try {
         value = ((double[]) this.getValue.invoke(this.fromObject))[index];
      } catch (Exception ignored) {
         // do nothing
      }

      return value;
   }
}
