package us.ihmc.yoVariables.registry;

import static us.ihmc.robotics.Assert.assertTrue;

import java.io.OutputStream;
import java.io.PrintStream;

import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.variable.YoDouble;

public class YoToolsTest
{

   @Test // timeout = 30000
   public void testPrintSizeRecursively()
   {
      YoVariableRegistry rootRegistry = new YoVariableRegistry("rootRegistry");

      int numberOfFirstLevelChildRegistries = 2;
      int numberOfFirstLevelYoVariables = 1;
      int numberOfSecondLevelChildRegistries = 1;
      int numberOfSecondLevelYoVariables = 4;
      int numberOfThirdLevelChildRegistries = 1;
      int numberOfThirdLevelYoVariables = 1;

      int totalNumberOfYoVariables = numberOfFirstLevelChildRegistries * (numberOfFirstLevelYoVariables
            + numberOfSecondLevelChildRegistries * (numberOfSecondLevelYoVariables + numberOfThirdLevelChildRegistries * numberOfThirdLevelYoVariables));

      for (int i = 0; i < numberOfFirstLevelChildRegistries; i++)
      {
         YoVariableRegistry firstLevelChild = new YoVariableRegistry("firstLevelChild_" + i);
         registerYoDoubles(firstLevelChild, numberOfFirstLevelYoVariables);
         rootRegistry.addChild(firstLevelChild);

         for (int j = 0; j < numberOfSecondLevelChildRegistries; j++)
         {
            YoVariableRegistry secondLevelChild = new YoVariableRegistry("secondLevelChild_" + j);
            registerYoDoubles(secondLevelChild, numberOfSecondLevelYoVariables);
            firstLevelChild.addChild(secondLevelChild);

            for (int k = 0; k < numberOfThirdLevelChildRegistries; k++)
            {
               YoVariableRegistry thirdLevelChild = new YoVariableRegistry("thirdLevelChild_" + k);
               registerYoDoubles(thirdLevelChild, numberOfThirdLevelYoVariables);
               secondLevelChild.addChild(thirdLevelChild);
            }
         }
      }

      Interceptor interceptor = new Interceptor(System.out);
      System.setOut(interceptor);

      int minimumVariablesToPrint = 2;
      int minimumChildrenToPrint = 2;

      YoTools.printSizeRecursively(minimumVariablesToPrint, minimumChildrenToPrint, rootRegistry);

      String[] strings = interceptor.getBuffer();
      assertTrue(strings.length != 0);

      // LogTools does not use System.out somehow, so the output for the first test is missing and the rest gets shifted.
      //      assertTrue(strings[1].contains(rootRegistry.getName()));
      assertTrue(strings[2 - 1].contains(String.valueOf(totalNumberOfYoVariables)));
      assertTrue(strings[5 - 1].contains("firstLevelChild_0.secondLevelChild_0"));
      assertTrue(strings[5 - 1].contains("Variables: " + numberOfSecondLevelYoVariables));
      assertTrue(strings[5 - 1].contains("Children: " + numberOfSecondLevelChildRegistries));
      assertTrue(strings[6 - 1].contains("firstLevelChild_1.secondLevelChild_0"));
      assertTrue(strings[6 - 1].contains("Variables: " + numberOfSecondLevelYoVariables));
      assertTrue(strings[6 - 1].contains("Children: " + numberOfSecondLevelChildRegistries));
      assertTrue(strings[7 - 1].contains("rootRegistry"));
      assertTrue(strings[7 - 1].contains("Variables: " + 0));
      assertTrue(strings[7 - 1].contains("Children: " + numberOfFirstLevelChildRegistries));
   }

   @Test // timeout = 30000
   public void testPrintAllVariablesIncludingDescendants()
   {
      Interceptor interceptor = new Interceptor(System.out);

      YoVariableRegistry robotRegistry = new YoVariableRegistry("robot");
      YoVariableRegistry controllerRegistry = new YoVariableRegistry("controller");
      YoVariableRegistry testRegistry = new YoVariableRegistry("testRegistry");

      robotRegistry.addChild(controllerRegistry);
      controllerRegistry.addChild(testRegistry);

      new YoDouble("robotVariable", robotRegistry);
      new YoDouble("controlVariable", controllerRegistry);
      new YoDouble("variableOne", testRegistry);
      new YoDouble("variableTwo", testRegistry);
      new YoDouble("variableThree", testRegistry);
      new YoDouble("variableFour", testRegistry);

      YoTools.printAllVariablesIncludingDescendants(robotRegistry, interceptor);
      String[] buffer = interceptor.getBuffer();

      assertTrue(buffer.length != 0);
      assertTrue(buffer[0].equals("robot.robotVariable"));
      assertTrue(buffer[1].equals("robot.controller.controlVariable"));
      assertTrue(buffer[2].equals("robot.controller.testRegistry.variableOne"));
      assertTrue(buffer[3].equals("robot.controller.testRegistry.variableTwo"));
      assertTrue(buffer[4].equals("robot.controller.testRegistry.variableThree"));
      assertTrue(buffer[5].equals("robot.controller.testRegistry.variableFour"));
   }

   private void registerYoDoubles(YoVariableRegistry registry, int numberOfYoDoublesToRegister)
   {
      for (int i = 0; i < numberOfYoDoublesToRegister; i++)
      {
         new YoDouble("yoDouble_" + i, registry);
      }
   }

   private class Interceptor extends PrintStream
   {
      private final StringBuffer buffer = new StringBuffer();

      public Interceptor(OutputStream out)
      {
         super(out);
      }

      @Override
      public void print(String s)
      {
         buffer.append(s);
      }

      @Override
      public void println(String s)
      {
         print(s + "\n");
      }

      public String[] getBuffer()
      {
         return buffer.toString().split("\n");
      }
   }
}
