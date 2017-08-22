package us.ihmc.yoVariables.variable;

import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.lang.reflect.Method;

public class YoIntegerFieldedArray extends YoIntegerFielded {
   private int index;

   public YoIntegerFieldedArray(String withName, YoVariableRegistry forRegistry, Object fromObject, Method getValue, int index) {
      super(withName, forRegistry, fromObject, getValue);

      this.index = index;
   }

   @Override
   public int getIntegerValue() {
      int value = Integer.MIN_VALUE;

      try {
         value = ((int[]) this.getValue.invoke(this.fromObject))[index];
      } catch (Exception ignored) {
         // do nothing
      }

      return value;
   }
}
