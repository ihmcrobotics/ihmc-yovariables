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

public class EnumParameterTest
{
   private enum TestEnum 
   {
      A, B, C, D, E, F, G, H, I, J, K, L, M;
   }
   
   
   private final static TestEnum initialValue = TestEnum.D;

   public EnumParameter<TestEnum> createParameterWithNamespace()
   {
      YoVariableRegistry root = new YoVariableRegistry("root");
      YoVariableRegistry a = new YoVariableRegistry("a");
      YoVariableRegistry b = new YoVariableRegistry("b");
      YoVariableRegistry c = new YoVariableRegistry("c");

      root.addChild(a);
      a.addChild(b);
      b.addChild(c);

      EnumParameter<TestEnum> param = new EnumParameter<>("param", c, TestEnum.class, true, initialValue);

      return param;
   }
   
   @Test(timeout = 1000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testLoadNullValue()
   {
      YoVariableRegistry dummy = new YoVariableRegistry("dummy");
      EnumParameter<TestEnum> yesnull = new EnumParameter<>("yesnull", dummy, TestEnum.class, true, initialValue);
      
      yesnull.load("NULL");
      
      assertEquals(null, yesnull.getValue());
      

   }
   
   @Test(timeout = 1000, expected = RuntimeException.class)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testDisallowNullLoadValue()
   {
      YoVariableRegistry dummy = new YoVariableRegistry("dummy");
      EnumParameter<TestEnum> nonull = new EnumParameter<>("nonull", dummy, TestEnum.class, false, initialValue);
      nonull.load("null");
      
   }
   
   @Test(timeout = 1000, expected = RuntimeException.class)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testDisallowNullConstructValue()
   {
      YoVariableRegistry dummy = new YoVariableRegistry("dummy");
      new EnumParameter<>("nonull", dummy, TestEnum.class, false, null);
      
   }

   @Test(timeout = 1000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testConstructDefaultValue()
   {
      YoVariableRegistry dummy = new YoVariableRegistry("dummy");
      EnumParameter<TestEnum> nonull = new EnumParameter<>("nonull", dummy, TestEnum.class, false);
      EnumParameter<TestEnum> yesnull = new EnumParameter<>("yesnull", dummy, TestEnum.class, true);
      
      nonull.loadDefault();
      yesnull.loadDefault();
      
      assertEquals(TestEnum.A, nonull.getValue());
      assertEquals(null, yesnull.getValue());
      
   }

   @Test(timeout = 1000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testGetNamespace()
   {

      EnumParameter<TestEnum> param = createParameterWithNamespace();

      assertEquals("root.a.b.c", param.getNameSpace().toString());
      assertEquals("param", param.getName());

   }
   
   @Test(timeout = 1000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testLoadFromString()
   {

      for(TestEnum element : TestEnum.values())
      { 
         YoVariableRegistry dummy = new YoVariableRegistry("dummy");
         EnumParameter<TestEnum> param = new EnumParameter<>("test", dummy, TestEnum.class, true, null);
         String stringValue = element.toString();
         param.load(stringValue);
         
         assertEquals(TestEnum.valueOf(stringValue), param.getValue());
         assertEquals(stringValue, param.getValueAsString());
      }
   }

   @Test(expected = RuntimeException.class, timeout = 1000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testGetBeforeLoad()
   {
      EnumParameter<TestEnum>param = createParameterWithNamespace();
      param.getValue();
   }

   @Test(timeout = 1000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testDefault()
   {
      EnumParameter<TestEnum>param = createParameterWithNamespace();
      param.loadDefault();
      assertEquals(initialValue, param.getValue());
   }

   @Test(timeout = 1000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testListener()
   {
      EnumParameter<TestEnum>param = createParameterWithNamespace();
      CallbackTest callback = new CallbackTest();
      param.addParameterChangedListener(callback);
      
      assertFalse(callback.set);
      
      param.loadDefault();
      
      callback.set = false;
      
      // No change
      param.getVariable().setValueFromDouble(param.getVariable().getValueAsDouble());
      assertFalse(callback.set);
      
      param.getVariable().setValueFromDouble(TestEnum.K.ordinal());
      
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