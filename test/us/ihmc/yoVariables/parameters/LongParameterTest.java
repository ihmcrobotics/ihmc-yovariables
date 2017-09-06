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

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.yoVariables.listener.ParameterChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class LongParameterTest
{
   private final static long initialValue = 42;

   public LongParameter createParameterWithNamespace()
   {
      YoVariableRegistry root = new YoVariableRegistry("root");
      YoVariableRegistry a = new YoVariableRegistry("a");
      YoVariableRegistry b = new YoVariableRegistry("b");
      YoVariableRegistry c = new YoVariableRegistry("c");

      root.addChild(a);
      a.addChild(b);
      b.addChild(c);

      LongParameter param = new LongParameter("param", c, initialValue);

      return param;
   }
   
   @Test(timeout = 1000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testConstructDefaultValue()
   {
      YoVariableRegistry dummy = new YoVariableRegistry("dummy");
      LongParameter test = new LongParameter("test", dummy);
      test.loadDefault();
      
      assertEquals(0, test.getValue());
      
   }


   @Test(timeout = 1000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testGetNamespace()
   {

      LongParameter param = createParameterWithNamespace();

      assertEquals("root.a.b.c", param.getNameSpace().toString());
      assertEquals("param", param.getName());

   }
   
   @Test(timeout = 1000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testLoadFromString()
   {

      for(long s = 100L * (long)Integer.MIN_VALUE; s < 100L * (long)Integer.MAX_VALUE; s+= (long)Integer.MAX_VALUE)
      { 
         YoVariableRegistry dummy = new YoVariableRegistry("dummy");
         LongParameter param = new LongParameter("test", dummy);
         param.load(String.valueOf(s));
         
         assertEquals(s, param.getValue());
         assertEquals(String.valueOf(s), param.getValueAsString());
      }
   }

   @Test(expected = RuntimeException.class, timeout = 1000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testGetBeforeLoad()
   {
      LongParameter param = createParameterWithNamespace();
      param.getValue();
   }

   @Test(timeout = 1000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testDefault()
   {
      LongParameter param = createParameterWithNamespace();
      param.loadDefault();
      assertEquals(initialValue, param.getValue());
   }

   @Test(timeout = 1000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
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
