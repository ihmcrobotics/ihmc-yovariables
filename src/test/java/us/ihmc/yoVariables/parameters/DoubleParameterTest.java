/*
 * Copyright 2017 Florida Institute for Human and Machine Cognition (IHMC)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package us.ihmc.yoVariables.parameters;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.listener.YoParameterChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DoubleParameterTest
{
   private final static double initialValue = 42.0;

   public DoubleParameter createParameterWithNamespace()
   {
      YoRegistry root = new YoRegistry("root");
      YoRegistry a = new YoRegistry("a");
      YoRegistry b = new YoRegistry("b");
      YoRegistry c = new YoRegistry("c");

      root.addChild(a);
      a.addChild(b);
      b.addChild(c);

      DoubleParameter param = new DoubleParameter("param", "paramDescription", c, initialValue, -20.3, 24.5);

      return param;
   }

   @Test // timeout = 1000
   public void testConstructDefaultValue()
   {
      YoRegistry dummy = new YoRegistry("dummy");
      DoubleParameter test = new DoubleParameter("test", dummy);
      test.loadDefault();

      assertTrue(Double.isNaN(test.getAsDouble()));

   }

   @Test // timeout = 1000
   public void testDuplicate()
   {
      DoubleParameter param = createParameterWithNamespace();
      YoDouble var = (YoDouble) param.getVariable();
      param.loadDefault();

      var.set(632.0);

      YoRegistry newRegistry = new YoRegistry("newRegistry");
      YoDouble newVar = var.duplicate(newRegistry);
      DoubleParameter newParam = (DoubleParameter) newVar.getParameter();

      assertEquals(param.getName(), newParam.getName());
      assertEquals(param.getDescription(), newParam.getDescription());
      assertEquals(param.getAsDouble(), newParam.getAsDouble(), 1e-9);
      assertEquals(var.getLowerBound(), newVar.getLowerBound(), 1e-9);
      assertEquals(var.getUpperBound(), newVar.getUpperBound(), 1e-9);

   }

   @Test // timeout = 1000
   public void testGetNamespace()
   {

      DoubleParameter param = createParameterWithNamespace();

      assertEquals("root.a.b.c", param.getNamespace().toString());
      assertEquals("param", param.getName());

   }

   @Test // timeout = 1000
   public void testLoadFromString()
   {

      for (double s = 0; s < 100.0; s += 1.153165)
      {
         YoRegistry dummy = new YoRegistry("dummy");
         DoubleParameter param = new DoubleParameter("test", dummy);
         param.load(String.valueOf(s));

         assertEquals(s, param.getAsDouble(), 1e-9);
         assertEquals(String.valueOf(s), param.getValueAsString());
      }
   }

   @Test // expected = RuntimeException.class, timeout = 1000
   public void testGetBeforeLoad()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         DoubleParameter param = createParameterWithNamespace();
         param.getAsDouble();
      });
   }

   @Test // timeout = 1000
   public void testDefault()
   {
      DoubleParameter param = createParameterWithNamespace();
      param.loadDefault();
      assertEquals(initialValue, param.getAsDouble(), 1e-9);
   }

   @Test // timeout = 1000
   public void testListener()
   {
      DoubleParameter param = createParameterWithNamespace();
      CallbackTest callback = new CallbackTest();
      param.addListener(callback);

      assertFalse(callback.set);

      param.loadDefault();

      callback.set = false;

      // No change
      param.getVariable().setValueFromDouble(param.getVariable().getValueAsDouble());
      assertFalse(callback.set);

      param.getVariable().setValueFromDouble(1.0);

      assertTrue(callback.set);

   }

   private class CallbackTest implements YoParameterChangedListener
   {
      boolean set = false;

      @Override
      public void changed(YoParameter v)
      {
         set = true;
      }

   }

}
