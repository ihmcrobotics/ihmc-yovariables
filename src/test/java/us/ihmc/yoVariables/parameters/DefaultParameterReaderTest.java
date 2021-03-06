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

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.registry.YoRegistry;

public class DefaultParameterReaderTest
{

   public static final double initialValue = 42.0;

   @Test // timeout = 1000
   public void testReadDefault()
   {
      YoRegistry root = new YoRegistry("root");
      YoRegistry a = new YoRegistry("a");

      root.addChild(a);
      DoubleParameter param = new DoubleParameter("param", a, initialValue);

      DefaultParameterReader reader = new DefaultParameterReader();
      reader.readParametersInRegistry(root);

      assertEquals(initialValue, param.getValue(), 1e-9);
   }
}
