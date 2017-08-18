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

import org.junit.Test;

import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ParameterLoaderTest
{
   
   private final static double initialValue = 42.0;

   
   
   @Test
   public void testLoadingFromRootRegistry()
   {
      YoVariableRegistry root = new YoVariableRegistry("root");
      YoVariableRegistry a = new YoVariableRegistry("a");
      YoVariableRegistry b = new YoVariableRegistry("b");
      YoVariableRegistry c = new YoVariableRegistry("c");

      root.addChild(a);
      a.addChild(b);
      b.addChild(c);

      DoubleParameter param = new DoubleParameter("param", c, initialValue);

      TestParameterLoader loader = new TestParameterLoader();
      loader.loadParametersInRegistry(root);
      
      
   }
   
   
   private static class TestParameterLoader extends AbstractParameterLoader
   {
      protected boolean hasValue(NameSpace namespace, String name)
      {
         System.out.println(namespace);
         System.out.println(name);
         return false;
      }
      
      protected String getValue(NameSpace namespace, String name)
      {
         throw new RuntimeException("Loader always returns false");
      }

   }
}
