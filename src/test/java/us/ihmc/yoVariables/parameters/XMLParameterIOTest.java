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

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.StringReader;
import java.nio.charset.Charset;

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

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testOverwritingDuringContruction() throws IOException
   {
      YoVariableRegistry target = new YoVariableRegistry("TestRegistry");
      DoubleParameter parameter1 = new DoubleParameter("TestParameter1", target);
      DoubleParameter parameter2 = new DoubleParameter("TestParameter2", target);

      String data1 = "<parameters>" + "<registry name=\"TestRegistry\">"
            + "<parameter name=\"TestParameter1\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.5\"/>"
            + "<parameter name=\"TestParameter2\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.5\"/>" + "</registry>" + "</parameters>";
      StringReader reader1 = new StringReader(data1);
      ReaderInputStream stream1 = new ReaderInputStream(reader1, Charset.forName("UTF-8"));

      String data2 = "<parameters>" + "<registry name=\"TestRegistry\">"
            + "<parameter name=\"TestParameter1\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.0\"/>" + "</registry>" + "</parameters>";
      StringReader reader2 = new StringReader(data2);
      ReaderInputStream stream2 = new ReaderInputStream(reader2, Charset.forName("UTF-8"));

      XmlParameterReader parameterReader = new XmlParameterReader(stream1, stream2);
      parameterReader.readParametersInRegistry(target);

      assertEquals(0.0, parameter1.getValue(), Double.MIN_VALUE);
      assertEquals(0.5, parameter2.getValue(), Double.MIN_VALUE);
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testOverwriting() throws IOException
   {
      YoVariableRegistry target = new YoVariableRegistry("TestRegistry");
      DoubleParameter parameter1 = new DoubleParameter("TestParameter1", target);
      DoubleParameter parameter2 = new DoubleParameter("TestParameter2", target);

      String data1 = "<parameters>" + "<registry name=\"TestRegistry\">"
            + "<parameter name=\"TestParameter1\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.5\"/>"
            + "<parameter name=\"TestParameter2\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.5\"/>" + "</registry>" + "</parameters>";
      StringReader reader1 = new StringReader(data1);
      ReaderInputStream stream1 = new ReaderInputStream(reader1, Charset.forName("UTF-8"));

      String data2 = "<parameters>" + "<registry name=\"TestRegistry\">"
            + "<parameter name=\"TestParameter1\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.0\"/>" + "</registry>" + "</parameters>";
      StringReader reader2 = new StringReader(data2);
      ReaderInputStream stream2 = new ReaderInputStream(reader2, Charset.forName("UTF-8"));

      XmlParameterReader parameterReader = new XmlParameterReader(stream1);
      parameterReader.overwrite(stream2);
      parameterReader.readParametersInRegistry(target);

      assertEquals(0.0, parameter1.getValue(), Double.MIN_VALUE);
      assertEquals(0.5, parameter2.getValue(), Double.MIN_VALUE);
   }

   @Test(timeout = 30000, expected = RuntimeException.class)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testOverwritingFails() throws IOException
   {
      YoVariableRegistry target = new YoVariableRegistry("TestRegistry");
      DoubleParameter parameter1 = new DoubleParameter("TestParameter1", target);
      DoubleParameter parameter2 = new DoubleParameter("TestParameter2", target);

      String data1 = "<parameters>" + "<registry name=\"TestRegistry\">"
            + "<parameter name=\"TestParameter2\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.5\"/>" + "</registry>" + "</parameters>";
      StringReader reader1 = new StringReader(data1);
      ReaderInputStream stream1 = new ReaderInputStream(reader1, Charset.forName("UTF-8"));

      String data2 = "<parameters>" + "<registry name=\"TestRegistry\">"
            + "<parameter name=\"TestParameter1\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.0\"/>" + "</registry>" + "</parameters>";
      StringReader reader2 = new StringReader(data2);
      ReaderInputStream stream2 = new ReaderInputStream(reader2, Charset.forName("UTF-8"));

      XmlParameterReader parameterReader = new XmlParameterReader(stream1);
      parameterReader.overwrite(stream2);
      parameterReader.readParametersInRegistry(target);

      assertEquals(0.0, parameter1.getValue(), Double.MIN_VALUE);
      assertEquals(0.5, parameter2.getValue(), Double.MIN_VALUE);
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testRootNamespaceDoesNotMatch() throws IOException
   {
      YoVariableRegistry target = new YoVariableRegistry("Root");
      DoubleParameter parameter = new DoubleParameter("TestParameter", target);

      String data1 = "<parameters>" + "<registry name=\"" + target.getName() + "\">" + "<parameter name=\"" + parameter.getName()
            + "\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.5\"/>" + "</registry>" + "</parameters>";
      StringReader reader1 = new StringReader(data1);
      ReaderInputStream stream1 = new ReaderInputStream(reader1, Charset.forName("UTF-8"));

      XmlParameterReader parameterReader = new XmlParameterReader("SomeOtherRoot", stream1);
      parameterReader.readParametersInRegistry(target);
      assertEquals(parameter.getLoadStatus(), ParameterLoadStatus.DEFAULT);
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   public void testRootNamespaceMatches() throws IOException
   {
      YoVariableRegistry target = new YoVariableRegistry("Root");
      DoubleParameter parameter = new DoubleParameter("TestParameter", target);

      String data1 = "<parameters>" + "<registry name=\"" + target.getName() + "\">" + "<parameter name=\"" + parameter.getName()
            + "\" type=\"DoubleParameter\" min=\"0.0\" max=\"1.0\" value=\"0.5\"/>" + "</registry>" + "</parameters>";
      StringReader reader1 = new StringReader(data1);
      ReaderInputStream stream1 = new ReaderInputStream(reader1, Charset.forName("UTF-8"));

      XmlParameterReader parameterReader = new XmlParameterReader(target.getName(), stream1);
      parameterReader.readParametersInRegistry(target);
      assertEquals(parameter.getLoadStatus(), ParameterLoadStatus.LOADED);
   }
}
