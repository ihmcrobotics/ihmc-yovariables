package us.ihmc.yoVariables;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.lang.annotation.Annotation;
import java.lang.reflect.Field;
import java.lang.reflect.Method;

public class YoVariablizer
{
   public void Yo(Object toYovariablize, String name, YoVariableRegistry forRegistry)
   {
      try
      {
         name = name.toLowerCase();

         for (Method m : toYovariablize.getClass().getMethods())
         {
            for (Annotation a : m.getAnnotations())
            {
               if (a.annotationType().equals(Yo.class))
               {
                  if (m.getParameterCount() == 0)
                  {
                     String fieldName = String.format("%s_%s", name, m.getName().toLowerCase().replace("get", ""));

                     if (m.getReturnType().equals(Double.class) || m.getReturnType().equals(double.class))
                     {
                        new YoDoubleFielded(fieldName, forRegistry, toYovariablize, m);
                     }
                     else if (m.getReturnType().equals(Integer.class) || m.getReturnType().equals(int.class))
                     {
                        new YoIntegerFielded(fieldName, forRegistry, toYovariablize, m);
                     }
                     else if (m.getReturnType().equals(Double[].class) || m.getReturnType().equals(double[].class))
                     {
                        if (((Yo) a).length() > 0)
                        {
                           for (int i = 0; i < ((Yo) a).length(); ++i)
                           {
                              new YoDoubleFieldedArray(String.format("%s_%d", fieldName, i), forRegistry, toYovariablize, m, i);
                           }
                        }
                     }
                     else if (m.getReturnType().equals(Integer[].class) || m.getReturnType().equals(int[].class))
                     {
                        if (((Yo) a).length() > 0)
                        {
                           for (int i = 0; i < ((Yo) a).length(); ++i)
                           {
                              new YoIntegerFieldedArray(String.format("%s_%d", fieldName, i), forRegistry, toYovariablize, m, i);
                           }
                        }
                     }
                     else
                     {
                        Yo(m.invoke(toYovariablize), fieldName, forRegistry);
                     }
                  }
                  else
                  {
                     throw new RuntimeException("method " + m.getName() + " must be getter (0 arguments) to use Yo annotation");
                  }
               }
            }
         }
      }
      catch (Exception e)
      {
         System.err.println("Could not yovariablize " + name + ": " + e.getMessage());
         e.printStackTrace();
      }
   }
}
