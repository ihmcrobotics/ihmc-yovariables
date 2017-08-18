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

import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class AbstractParameterLoaderTest
{

   private final static double initialValue = 42.0;

   @Test
   public void testLoadingNamespacesRegistry()
   {

      for (int i = 0; i < 4; i++)
      {
         YoVariableRegistry[] regs = {new YoVariableRegistry("root"), new YoVariableRegistry("a"), new YoVariableRegistry("b"), new YoVariableRegistry("c")};

         String[] expectedNamespaces = {"root.a.b.c", "a.b.c", "b.c", "c"};

         regs[0].addChild(regs[1]);
         regs[1].addChild(regs[2]);
         regs[2].addChild(regs[3]);

         new DoubleParameter("paramA", regs[3], initialValue);
         new DoubleParameter("paramB", regs[3], initialValue);
         new DoubleParameter("paramC", regs[3], initialValue);
         new DoubleParameter("paramD", regs[3], initialValue);

         TestParameterLoaderNamespace loaderRoot = new TestParameterLoaderNamespace(expectedNamespaces[i]);
         loaderRoot.loadParametersInRegistry(regs[i]);
      }

   }
   
   @Test(expected=RuntimeException.class)
   public void testDoubleLoad()
   {
      YoVariableRegistry root = new YoVariableRegistry("root");
      YoVariableRegistry a = new YoVariableRegistry("a");
      
      root.addChild(a);
      new DoubleParameter("param", a, initialValue);
      
      TestDefaultLoaderNamespace loader = new TestDefaultLoaderNamespace();
      loader.loadParametersInRegistry(root);
      loader.loadParametersInRegistry(a);
   }


   
   
   private static class TestParameterLoaderNamespace extends AbstractParameterLoader
   {
      private final String expectedNamespace;

      private TestParameterLoaderNamespace(String expectedNamespace)
      {
         this.expectedNamespace = expectedNamespace;
      }

      protected boolean hasValue(NameSpace namespace, String name)
      {
         assertEquals(expectedNamespace, namespace.getName());
         return false;
      }

      protected String getValue(NameSpace namespace, String name)
      {
         throw new RuntimeException("Loader always returns false");
      }

   }
   
   private static class TestDefaultLoaderNamespace extends AbstractParameterLoader
   {
      protected boolean hasValue(NameSpace namespace, String name)
      {
         return false;
      }

      protected String getValue(NameSpace namespace, String name)
      {
         throw new RuntimeException("Loader always returns false");
      }

   }
}
