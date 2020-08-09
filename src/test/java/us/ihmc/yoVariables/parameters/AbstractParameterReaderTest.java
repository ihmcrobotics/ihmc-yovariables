/*
 * Copyright 2017 Florida Institute for Human and Machine Cognition (IHMC) Licensed under the Apache
 * License, Version 2.0 (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License
 * is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
 * or implied. See the License for the specific language governing permissions and limitations under
 * the License.
 */
package us.ihmc.yoVariables.parameters;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.registry.YoRegistry;

public class AbstractParameterReaderTest
{
   private final static Random random = new Random(9492L);

   @Test // timeout = 1000
   public void testReadingNamespacesRegistry()
   {
      for (int i = 0; i < 4; i++)
      {
         YoRegistry[] regs = {new YoRegistry("root"), new YoRegistry("a"), new YoRegistry("b"), new YoRegistry("c")};

         String[] expectedNamespaces = {"root.a.b.c", "a.b.c", "b.c", "c"};

         regs[0].addChild(regs[1]);
         regs[1].addChild(regs[2]);
         regs[2].addChild(regs[3]);

         DoubleParameter parameter = new DoubleParameter("parameter", regs[3], random.nextDouble());
         int numberOfDefaults = random.nextInt(10);
         for (int defaultIdx = 0; defaultIdx < numberOfDefaults; defaultIdx++)
         {
            new DoubleParameter("Default" + defaultIdx, regs[3], 0.0);
         }

         Map<String, ParameterData> values = new HashMap<>();
         double expectedValue = random.nextDouble();
         values.put(expectedNamespaces[i] + "." + parameter.getName(), new ParameterData(Double.toString(expectedValue)));

         int numberOfUnmatched = random.nextInt(10);
         for (int unmatchedIdx = 0; unmatchedIdx < numberOfUnmatched; unmatchedIdx++)
         {
            values.put("Unmatched" + unmatchedIdx, new ParameterData(Double.toString(0.0)));
         }

         TestParameterReader readerRoot = new TestParameterReader(values);
         Set<String> defaultParameters = new HashSet<>();
         Set<String> unmatchedParameters = new HashSet<>();
         readerRoot.readParametersInRegistry(regs[i], defaultParameters, unmatchedParameters);

         assertEquals(numberOfDefaults, defaultParameters.size());
         assertEquals(numberOfUnmatched, unmatchedParameters.size());
         assertEquals(expectedValue, parameter.getAsDouble(), Double.MIN_VALUE);
      }
   }

   @Test // timeout = 1000
   public void testDoubleRead()
   {
      YoRegistry root = new YoRegistry("root");
      YoRegistry a = new YoRegistry("a");

      root.addChild(a);
      DoubleParameter parameter = new DoubleParameter("param", a, random.nextDouble());

      double loadedValue = random.nextDouble();

      Map<String, ParameterData> values = new HashMap<>();
      values.put("root.a.param", new ParameterData(Double.toString(loadedValue)));
      TestParameterReader reader = new TestParameterReader(values);

      reader.readParametersInRegistry(new YoRegistry("RandomRegistry"));
      reader.readParametersInRegistry(root);
      assertEquals(loadedValue, parameter.getAsDouble(), Double.MIN_VALUE);
   }

   private static class TestParameterReader extends AbstractParameterReader
   {
      private final Map<String, ParameterData> values;

      private TestParameterReader(Map<String, ParameterData> values)
      {
         this.values = values;
      }

      @Override
      protected Map<String, ParameterData> getValues()
      {
         return Collections.unmodifiableMap(values);
      }

   }

}
