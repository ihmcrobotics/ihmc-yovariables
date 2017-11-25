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

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.StringReader;

import org.apache.commons.io.input.ReaderInputStream;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class XMLParameterIOTest
{
   

   public static YoVariableRegistry createRegistries()
   {
      YoVariableRegistry[] regs = {new YoVariableRegistry("root"), new YoVariableRegistry("a"), new YoVariableRegistry("b"), new YoVariableRegistry("c")};

      regs[0].addChild(regs[1]);
      regs[1].addChild(regs[2]);
      regs[2].addChild(regs[3]);

      new DoubleParameter("paramA", "parameter A description", regs[3], 0);
      new DoubleParameter("paramB", regs[3], 0);
      new DoubleParameter("paramC", regs[3], 0);
      new DoubleParameter("paramD", regs[3], 0);
      new DoubleParameter("paramE", regs[2], 0);
      new DoubleParameter("paramF", regs[1], 0);
      new DoubleParameter("paramG", regs[0], 0);
      return regs[0];
   }

   
   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testEmptyFile() throws IOException
   {
      YoVariableRegistry target = createRegistries();
      String data = "<parameters/>";
      StringReader reader = new StringReader(data);
      
      
      XmlParameterReader parameterReader = new XmlParameterReader(new ReaderInputStream(reader));
      parameterReader.readParametersInRegistry(target);
   }
   
   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testWritingAndReading() throws IOException
   {
      YoVariableRegistry source = createRegistries();
      YoVariableRegistry target = createRegistries();
      
      DefaultParameterReader defaultReader = new DefaultParameterReader();
      defaultReader.readParametersInRegistry(source);
      
      source.getVariable("root.a.b.c.paramA").setValueFromDouble(1.0);
      source.getVariable("root.a.b.c.paramB").setValueFromDouble(2.0);
      source.getVariable("root.a.b.c.paramC").setValueFromDouble(3.0);
      source.getVariable("root.a.b.c.paramD").setValueFromDouble(4.0);
      source.getVariable("root.a.b.paramE").setValueFromDouble(5.0);
      source.getVariable("root.a.paramF").setValueFromDouble(6.0);
      source.getVariable("root.paramG").setValueFromDouble(7.0);
      
      
      ByteArrayOutputStream os = new ByteArrayOutputStream();
      
      XmlParameterWriter writer = new XmlParameterWriter();
      writer.writeParametersInRegistry(source);
      writer.write(os);
      
      
      XmlParameterReader reader = new XmlParameterReader(new ByteArrayInputStream(os.toByteArray()));
      reader.readParametersInRegistry(target);
      
      assertEquals(source.getVariable("root.a.b.c.paramA").getValueAsDouble(), target.getVariable("root.a.b.c.paramA").getValueAsDouble(), 1e-9);
      assertEquals(source.getVariable("root.a.b.c.paramB").getValueAsDouble(), target.getVariable("root.a.b.c.paramB").getValueAsDouble(), 1e-9);
      assertEquals(source.getVariable("root.a.b.c.paramC").getValueAsDouble(), target.getVariable("root.a.b.c.paramC").getValueAsDouble(), 1e-9);
      assertEquals(source.getVariable("root.a.b.c.paramD").getValueAsDouble(), target.getVariable("root.a.b.c.paramD").getValueAsDouble(), 1e-9);
      assertEquals(source.getVariable("root.a.b.paramE").getValueAsDouble(), target.getVariable("root.a.b.paramE").getValueAsDouble(), 1e-9);
      assertEquals(source.getVariable("root.a.paramF").getValueAsDouble(), target.getVariable("root.a.paramF").getValueAsDouble(), 1e-9);
      assertEquals(source.getVariable("root.paramG").getValueAsDouble(), target.getVariable("root.paramG").getValueAsDouble(), 1e-9);
   }
}
