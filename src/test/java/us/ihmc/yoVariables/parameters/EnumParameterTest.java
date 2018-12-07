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

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.parallel.Execution;
import org.junit.jupiter.api.parallel.ExecutionMode;

import us.ihmc.yoVariables.listener.ParameterChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

@Execution(ExecutionMode.SAME_THREAD)
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

      EnumParameter<TestEnum> param = new EnumParameter<>("param", "paramDescription", c, TestEnum.class, true, initialValue);

      return param;
   }
   
   @Test// timeout = 1000
   public void testLoadNullValue()
   {
      YoVariableRegistry dummy = new YoVariableRegistry("dummy");
      EnumParameter<TestEnum> yesnull = new EnumParameter<>("yesnull", dummy, TestEnum.class, true, initialValue);
      
      yesnull.load("NULL");
      
      assertEquals(null, yesnull.getValue());
      

   }
   

   @Test// timeout = 1000
   public void testDuplicate()
   {
      EnumParameter<TestEnum> param = createParameterWithNamespace();
      @SuppressWarnings("unchecked")
      YoEnum<TestEnum> var = (YoEnum<TestEnum>) param.getVariable();
      param.loadDefault();
      
      var.set(TestEnum.E);
      
      YoVariableRegistry newRegistry = new YoVariableRegistry("newRegistry");
      YoEnum<TestEnum> newVar = var.duplicate(newRegistry);
      @SuppressWarnings("unchecked")
      EnumParameter<TestEnum> newParam = (EnumParameter<TestEnum>) newVar.getParameter();
      
      assertEquals(param.getName(), newParam.getName());
      assertEquals(param.getDescription(), newParam.getDescription());
      assertEquals(param.getValue(), newParam.getValue());
      
      assertEquals(var.getAllowNullValue(), newVar.getAllowNullValue());
      assertArrayEquals(var.getEnumValues(), newVar.getEnumValues());
      
   }
   
   @Test// timeout = 1000
   public void testStringDuplicate()
   {
      YoVariableRegistry root = new YoVariableRegistry("root");

      EnumParameter<?> param = new EnumParameter<>("testString", "stringDescription", root, true, "A", "B", "C", "D", "E", "F");
      @SuppressWarnings("unchecked")
      YoEnum<TestEnum> var = (YoEnum<TestEnum>) param.getVariable();
      param.loadDefault();
      
      var.set(4);
      
      YoVariableRegistry newRegistry = new YoVariableRegistry("newRegistry");
      YoEnum<TestEnum> newVar = var.duplicate(newRegistry);
      @SuppressWarnings("unchecked")
      EnumParameter<TestEnum> newParam = (EnumParameter<TestEnum>) newVar.getParameter();
      
      assertEquals(param.getName(), newParam.getName());
      assertEquals(param.getDescription(), newParam.getDescription());
      assertEquals(param.getValueAsString(), newParam.getValueAsString());
      
      assertEquals(var.isBackedByEnum(), newVar.isBackedByEnum());
      assertEquals(var.getAllowNullValue(), newVar.getAllowNullValue());
      assertArrayEquals(var.getEnumValuesAsString(), newVar.getEnumValuesAsString());
      
   }
   
   @Test// timeout = 1000, expected = RuntimeException.class
   public void testDisallowNullLoadValue()
   {
      YoVariableRegistry dummy = new YoVariableRegistry("dummy");
      EnumParameter<TestEnum> nonull = new EnumParameter<>("nonull", dummy, TestEnum.class, false, initialValue);
      nonull.load("null");
      
   }
   
   @Test// timeout = 1000, expected = RuntimeException.class
   public void testDisallowNullConstructValue()
   {
      YoVariableRegistry dummy = new YoVariableRegistry("dummy");
      new EnumParameter<>("nonull", dummy, TestEnum.class, false, null);
      
   }

   @Test// timeout = 1000
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

   @Test// timeout = 1000
   public void testGetNamespace()
   {

      EnumParameter<TestEnum> param = createParameterWithNamespace();

      assertEquals("root.a.b.c", param.getNameSpace().toString());
      assertEquals("param", param.getName());

   }
   
   @Test// timeout = 1000
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

   @Test// expected = RuntimeException.class, timeout = 1000
   public void testGetBeforeLoad()
   {
      EnumParameter<TestEnum>param = createParameterWithNamespace();
      param.getValue();
   }

   @Test// timeout = 1000
   public void testDefault()
   {
      EnumParameter<TestEnum>param = createParameterWithNamespace();
      param.loadDefault();
      assertEquals(initialValue, param.getValue());
   }

   @Test// timeout = 1000
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
   
   @Test// timeout = 1000
   public void testStringBased()
   {
      String[] constants = { "A", "B", "C", "D", "E", "F", "G", "H" };
      
      YoVariableRegistry registry = new YoVariableRegistry("test");
      
      EnumParameter<?> nullDefault = new EnumParameter<>("nullDefault", "", registry, true, constants);
      nullDefault.loadDefault();
      assertEquals("null", nullDefault.getValueAsString());

      
      EnumParameter<?> constantDefault = new EnumParameter<>("constantDefault", "", registry, false, constants);
      constantDefault.loadDefault();
      assertEquals("A", constantDefault.getValueAsString());
      
      
      for(String c : constants)
      {
         constantDefault.setToString(c);
         assertEquals(c, constantDefault.getValueAsString());
         
      }
      
      nullDefault.setToString("A");
      assertEquals("A", nullDefault.getValueAsString());
      
      nullDefault.setToString("NULL");
      assertEquals("null", nullDefault.getValueAsString());
      
      
   }
   
   @Test// expected = RuntimeException.class, timeout = 1000
   public void testStringBasedAccess()
   {
      String[] constants = { "A", "B", "C", "D", "E", "F", "G", "H" };
      
      YoVariableRegistry registry = new YoVariableRegistry("test");
      EnumParameter<?> nullDefault = new EnumParameter<>("nullDefault", "", registry, true, constants);
      nullDefault.loadDefault();
      nullDefault.getValue();
   }
   
   @Test// expected = RuntimeException.class, timeout = 1000
   public void testStringBasedAccessSetNull()
   {
      String[] constants = { "A", "B", "C", "D", "E", "F", "G", "H" };
      
      YoVariableRegistry registry = new YoVariableRegistry("test");

      EnumParameter<?> constantDefault = new EnumParameter<>("constantDefault", "", registry, false, constants);
      constantDefault.loadDefault();
      
      
      constantDefault.setToString("null");
   }
   
   @Test// expected = RuntimeException.class, timeout = 1000
   public void testStringBasedAccessSetNonExistant()
   {
      String[] constants = { "A", "B", "C", "D", "E", "F", "G", "H" };
      
      YoVariableRegistry registry = new YoVariableRegistry("test");
      
      EnumParameter<?> constantDefault = new EnumParameter<>("constantDefault", "", registry, false, constants);
      constantDefault.loadDefault();
      
      
      constantDefault.setToString("NONEXISTANT");
   }

   @Test// expected = RuntimeException.class, timeout = 1000
   public void testStringBasedAccessNullValueConstant()
   {
      String[] constants = { "A", "B", "C", "NuLl", "E", "F", "G", "H" };
      
      YoVariableRegistry registry = new YoVariableRegistry("test");
      
      new EnumParameter<>("constantDefault", "", registry, false, constants);
      
   }

   @Test// expected = RuntimeException.class, timeout = 1000
   public void testStringBasedAccessNullConstant()
   {
      String[] constants = {"A", "B", "C", null, "E", "F", "G", "H"};

      YoVariableRegistry registry = new YoVariableRegistry("test");

      new EnumParameter<>("constantDefault", "", registry, false, constants);

   }

   @Test// expected = RuntimeException.class, timeout = 1000
   public void testStringBasedAccessEmptyConstantListNotNull()
   {
      String[] constants = {};

      YoVariableRegistry registry = new YoVariableRegistry("test");

      new EnumParameter<>("constantDefault", "", registry, false, constants);

   }

   @Test// timeout = 1000
   public void testStringBasedAccessEmptyConstantList()
   {
      String[] constants = {};

      YoVariableRegistry registry = new YoVariableRegistry("test");

      EnumParameter<?> nullDefault = new EnumParameter<>("nullDefault", "", registry, true, constants);
      nullDefault.loadDefault();
      assertEquals("null", nullDefault.getValueAsString());

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
