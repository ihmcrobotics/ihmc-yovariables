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

import static org.junit.Assert.*;

import org.junit.Test;

import us.ihmc.yoVariables.listener.ParameterChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class DoubleParameterTest
{
   private final static double initialValue = 42.0;

   public DoubleParameter createParameterWithNamespace()
   {
      YoVariableRegistry root = new YoVariableRegistry("root");
      YoVariableRegistry a = new YoVariableRegistry("a");
      YoVariableRegistry b = new YoVariableRegistry("b");
      YoVariableRegistry c = new YoVariableRegistry("c");

      root.addChild(a);
      a.addChild(b);
      b.addChild(c);

      DoubleParameter param = new DoubleParameter("param", c, initialValue);

      return param;
   }

   @Test
   public void testGetNamespace()
   {

      DoubleParameter param = createParameterWithNamespace();

      assertEquals("root.a.b.c", param.getNameSpace().toString());
      assertEquals("param", param.getName());

   }

   @Test(expected = RuntimeException.class)
   public void testGetBeforeLoad()
   {
      DoubleParameter param = createParameterWithNamespace();
      param.getValue();
   }

   @Test
   public void testDefault()
   {
      DoubleParameter param = createParameterWithNamespace();
      param.loadDefault();
      assertEquals(initialValue, param.getValue(), 1e-9);
   }

   @Test
   public void testListener()
   {
      DoubleParameter param = createParameterWithNamespace();
      CallbackTest callback = new CallbackTest();
      param.addParameterChangedListener(callback);
      
      assertFalse(callback.set);
      
      param.loadDefault();
      
      callback.set = false;
      
      param.getVariable().setValueFromDouble(1.0);
      
      assertTrue(callback.set);
      
      
      
   }

   private class CallbackTest implements ParameterChangedListener
   {
      boolean set = false;

      @Override
      public void variableChanged(YoParameter<?> v)
      {
         set = true;
      }

   }

}
