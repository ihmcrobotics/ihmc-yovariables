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

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.yoVariables.listener.ParameterChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class BooleanParameterTest
{
   private final static boolean initialValue = true;

   public BooleanParameter createParameterWithNamespace()
   {
      YoVariableRegistry root = new YoVariableRegistry("root");
      YoVariableRegistry a = new YoVariableRegistry("a");
      YoVariableRegistry b = new YoVariableRegistry("b");
      YoVariableRegistry c = new YoVariableRegistry("c");

      root.addChild(a);
      a.addChild(b);
      b.addChild(c);

      BooleanParameter param = new BooleanParameter("param", "paramDescription", c, initialValue);

      return param;
   }

   @Test(timeout = 1000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testGetNamespace()
   {

      BooleanParameter param = createParameterWithNamespace();

      assertEquals("root.a.b.c", param.getNameSpace().toString());
      assertEquals("param", param.getName());

   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(expected = RuntimeException.class, timeout = 1000)
   public void testGetBeforeLoad()
   {
      BooleanParameter param = createParameterWithNamespace();
      param.getValue();
   }
   
   @Test(timeout = 1000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testLoadFromString()
   {

      String options[] = { "false", "true", "FALSE", "TRUE", "False", "True" };
      
      for(int i = 0; i < options.length; i++)
      { 
         
         YoVariableRegistry dummy = new YoVariableRegistry("dummy");
         BooleanParameter param = new BooleanParameter("test", dummy);
         param.load(options[i]);
         
         assertEquals(i % 2 == 1, param.getValue());
         assertEquals(options[i].toLowerCase(), param.getValueAsString());
      }
   }

   @Test(timeout = 1000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testDefault()
   {
      BooleanParameter param = createParameterWithNamespace();
      param.loadDefault();
      assertEquals(initialValue, param.getValue());
   }

   @Test(timeout = 1000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testDuplicate()
   {
      BooleanParameter param = createParameterWithNamespace();
      YoBoolean var = (YoBoolean) param.getVariable();
      param.loadDefault();
      
      var.set(true);
      
      YoVariableRegistry newRegistry = new YoVariableRegistry("newRegistry");
      YoBoolean newVar = var.duplicate(newRegistry);
      BooleanParameter newParam = (BooleanParameter) newVar.getParameter();
      
      assertEquals(param.getName(), newParam.getName());
      assertEquals(param.getDescription(), newParam.getDescription());
      assertEquals(param.getValue(), newParam.getValue());
      
      
   }
   
   @Test(timeout = 1000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testListener()
   {
      BooleanParameter param = createParameterWithNamespace();
      CallbackTest callback = new CallbackTest();
      param.addParameterChangedListener(callback);
      
      assertFalse(callback.set);
      
      param.loadDefault();
      
      callback.set = false;
      
      // No change
      param.getVariable().setValueFromDouble(param.getVariable().getValueAsDouble());
      assertFalse(callback.set);
      
      
      param.getVariable().setValueFromDouble(0.0);
      
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
