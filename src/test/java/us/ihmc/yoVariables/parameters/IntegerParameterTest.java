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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.yoVariables.listener.ParameterChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class IntegerParameterTest
{
   private final static int initialValue = 42;

   public IntegerParameter createParameterWithNamespace()
   {
      YoVariableRegistry root = new YoVariableRegistry("root");
      YoVariableRegistry a = new YoVariableRegistry("a");
      YoVariableRegistry b = new YoVariableRegistry("b");
      YoVariableRegistry c = new YoVariableRegistry("c");

      root.addChild(a);
      a.addChild(b);
      b.addChild(c);

      IntegerParameter param = new IntegerParameter("param", "paramDescription", c, initialValue, 10, 30);

      return param;
   }
   
   @Test(timeout = 1000)
   public void testConstructDefaultValue()
   {
      YoVariableRegistry dummy = new YoVariableRegistry("dummy");
      IntegerParameter test = new IntegerParameter("test", dummy);
      test.loadDefault();
      
      assertEquals(0, test.getValue());
      
   }

   @Test(timeout = 1000)
   public void testDuplicate()
   {
      IntegerParameter param = createParameterWithNamespace();
      YoInteger var = (YoInteger) param.getVariable();
      param.loadDefault();
      
      var.set(632);
      
      YoVariableRegistry newRegistry = new YoVariableRegistry("newRegistry");
      YoInteger newVar = var.duplicate(newRegistry);
      IntegerParameter newParam = (IntegerParameter) newVar.getParameter();
      
      assertEquals(param.getName(), newParam.getName());
      assertEquals(param.getDescription(), newParam.getDescription());
      assertEquals(param.getValue(), newParam.getValue(), 1e-9);
      assertEquals(var.getManualScalingMin(), newVar.getManualScalingMin(), 1e-9);
      assertEquals(var.getManualScalingMax(), newVar.getManualScalingMax(), 1e-9);
      
      
      
   }

   @Test(timeout = 1000)
   public void testGetNamespace()
   {

      IntegerParameter param = createParameterWithNamespace();

      assertEquals("root.a.b.c", param.getNameSpace().toString());
      assertEquals("param", param.getName());

   }
   
   @Test(timeout = 1000)
   public void testLoadFromString()
   {

      for(int s = - 100; s < 100.0; s++)
      { 
         YoVariableRegistry dummy = new YoVariableRegistry("dummy");
         IntegerParameter param = new IntegerParameter("test", dummy);
         param.load(String.valueOf(s));
         
         assertEquals(s, param.getValue());
         assertEquals(String.valueOf(s), param.getValueAsString());
      }
   }

   @Test(expected = RuntimeException.class, timeout = 1000)
   public void testGetBeforeLoad()
   {
      IntegerParameter param = createParameterWithNamespace();
      param.getValue();
   }

   @Test(timeout = 1000)
   public void testDefault()
   {
      IntegerParameter param = createParameterWithNamespace();
      param.loadDefault();
      assertEquals(initialValue, param.getValue());
   }

   @Test(timeout = 1000)
   public void testListener()
   {
      IntegerParameter param = createParameterWithNamespace();
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

   private class CallbackTest implements ParameterChangedListener
   {
      boolean set = false;

      @Override
      public void notifyOfParameterChange(YoParameter<?> v)
      {
         set = true;
      }

   }

}
