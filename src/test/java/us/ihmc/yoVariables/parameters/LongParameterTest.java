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

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.parallel.Execution;
import org.junit.jupiter.api.parallel.ExecutionMode;

import us.ihmc.yoVariables.listener.YoParameterChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

@Execution(ExecutionMode.SAME_THREAD)
public class LongParameterTest
{
   private final static long initialValue = 42;

   public LongParameter createParameterWithNamespace()
   {
      YoRegistry root = new YoRegistry("root");
      YoRegistry a = new YoRegistry("a");
      YoRegistry b = new YoRegistry("b");
      YoRegistry c = new YoRegistry("c");

      root.addChild(a);
      a.addChild(b);
      b.addChild(c);

      LongParameter param = new LongParameter("param", "paramDescription", c, initialValue, -102024, 294);

      return param;
   }

   @Test // timeout = 1000
   public void testConstructDefaultValue()
   {
      YoRegistry dummy = new YoRegistry("dummy");
      LongParameter test = new LongParameter("test", dummy);
      test.loadDefault();

      assertEquals(0, test.getValue());

   }

   @Test // timeout = 1000
   public void testDuplicate()
   {
      LongParameter param = createParameterWithNamespace();
      YoLong var = (YoLong) param.getVariable();
      param.loadDefault();

      var.set(632);

      YoRegistry newRegistry = new YoRegistry("newRegistry");
      YoLong newVar = var.duplicate(newRegistry);
      LongParameter newParam = (LongParameter) newVar.getParameter();

      assertEquals(param.getName(), newParam.getName());
      assertEquals(param.getDescription(), newParam.getDescription());
      assertEquals(param.getValue(), newParam.getValue(), 1e-9);
      assertEquals(var.getLowerBound(), newVar.getLowerBound(), 1e-9);
      assertEquals(var.getUpperBound(), newVar.getUpperBound(), 1e-9);

   }

   @Test // timeout = 1000
   public void testGetNamespace()
   {

      LongParameter param = createParameterWithNamespace();

      assertEquals("root.a.b.c", param.getNameSpace().toString());
      assertEquals("param", param.getName());

   }

   @Test // timeout = 1000
   public void testLoadFromString()
   {

      for (long s = 100L * Integer.MIN_VALUE; s < 100L * Integer.MAX_VALUE; s += Integer.MAX_VALUE)
      {
         YoRegistry dummy = new YoRegistry("dummy");
         LongParameter param = new LongParameter("test", dummy);
         param.load(String.valueOf(s));

         assertEquals(s, param.getValue());
         assertEquals(String.valueOf(s), param.getValueAsString());
      }
   }

   @Test // expected = RuntimeException.class, timeout = 1000
   public void testGetBeforeLoad()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         LongParameter param = createParameterWithNamespace();
         param.getValue();
      });
   }

   @Test // timeout = 1000
   public void testDefault()
   {
      LongParameter param = createParameterWithNamespace();
      param.loadDefault();
      assertEquals(initialValue, param.getValue());
   }

   @Test // timeout = 1000
   public void testListener()
   {
      LongParameter param = createParameterWithNamespace();
      CallbackTest callback = new CallbackTest();
      param.addParameterChangedListener(callback);

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
