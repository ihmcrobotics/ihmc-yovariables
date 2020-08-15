package us.ihmc.yoVariables.registry;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.OutputStream;
import java.io.PrintStream;

import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.exceptions.IllegalNameException;
import us.ihmc.yoVariables.tools.YoTools;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoToolsTest
{
   @Test
   public void testIllegalCharacters()
   {
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc`abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc~abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc!abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc@abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc#abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc$abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc%abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc^abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc&abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc*abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc(abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc)abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc=abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc+abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc{abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc}abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc|abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc\\abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc'abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc\"abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc,abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc.abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc?abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc:abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc;abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc<abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc>abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc[abc"));
      assertThrows(IllegalNameException.class, () -> YoTools.checkForIllegalCharacters("abc]abc"));

      assertDoesNotThrow(() -> YoTools.checkForIllegalCharacters("abcabc"));
      assertDoesNotThrow(() -> YoTools.checkForIllegalCharacters("abc_abc"));
      assertDoesNotThrow(() -> YoTools.checkForIllegalCharacters("abc-abc"));
      assertDoesNotThrow(() -> YoTools.checkForIllegalCharacters("abc0abc"));
      assertDoesNotThrow(() -> YoTools.checkForIllegalCharacters("abc1abc"));
      assertDoesNotThrow(() -> YoTools.checkForIllegalCharacters("abc2abc"));
      assertDoesNotThrow(() -> YoTools.checkForIllegalCharacters("abc3abc"));
      assertDoesNotThrow(() -> YoTools.checkForIllegalCharacters("abc4abc"));
      assertDoesNotThrow(() -> YoTools.checkForIllegalCharacters("abc5abc"));
      assertDoesNotThrow(() -> YoTools.checkForIllegalCharacters("abc6abc"));
      assertDoesNotThrow(() -> YoTools.checkForIllegalCharacters("abc7abc"));
      assertDoesNotThrow(() -> YoTools.checkForIllegalCharacters("abc8abc"));
      assertDoesNotThrow(() -> YoTools.checkForIllegalCharacters("abc9abc"));
      assertDoesNotThrow(() -> YoTools.checkForIllegalCharacters("abcAabc"));
   }

   @Test // timeout = 30000
   public void testPrintSizeRecursively()
   {
      YoRegistry rootRegistry = new YoRegistry("rootRegistry");

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
         YoRegistry firstLevelChild = new YoRegistry("firstLevelChild_" + i);
         registerYoDoubles(firstLevelChild, numberOfFirstLevelYoVariables);
         rootRegistry.addChild(firstLevelChild);

         for (int j = 0; j < numberOfSecondLevelChildRegistries; j++)
         {
            YoRegistry secondLevelChild = new YoRegistry("secondLevelChild_" + j);
            registerYoDoubles(secondLevelChild, numberOfSecondLevelYoVariables);
            firstLevelChild.addChild(secondLevelChild);

            for (int k = 0; k < numberOfThirdLevelChildRegistries; k++)
            {
               YoRegistry thirdLevelChild = new YoRegistry("thirdLevelChild_" + k);
               registerYoDoubles(thirdLevelChild, numberOfThirdLevelYoVariables);
               secondLevelChild.addChild(thirdLevelChild);
            }
         }
      }

      Interceptor interceptor = new Interceptor(System.out);
      System.setOut(interceptor);

      int minimumVariablesToPrint = 2;
      int minimumChildrenToPrint = 2;

      YoTools.printStatistics(minimumVariablesToPrint, minimumChildrenToPrint, rootRegistry);

      String[] strings = interceptor.getBuffer();
      assertTrue(strings.length != 0);

      // LogTools does not use System.out somehow, so the output for the first test is missing and the rest gets shifted.
      //      assertTrue(strings[1].contains(rootRegistry.getName()));
      assertTrue(strings[1].contains(String.valueOf(totalNumberOfYoVariables)));
      assertTrue(strings[3].contains("firstLevelChild_0.secondLevelChild_0"));
      assertTrue(strings[3].contains("Variables: " + numberOfSecondLevelYoVariables));
      assertTrue(strings[3].contains("Children: " + numberOfSecondLevelChildRegistries));
      assertTrue(strings[4].contains("firstLevelChild_1.secondLevelChild_0"));
      assertTrue(strings[4].contains("Variables: " + numberOfSecondLevelYoVariables));
      assertTrue(strings[4].contains("Children: " + numberOfSecondLevelChildRegistries));
      assertTrue(strings[5].contains("rootRegistry"));
      assertTrue(strings[5].contains("Variables: " + 0));
      assertTrue(strings[5].contains("Children: " + numberOfFirstLevelChildRegistries));
   }

   private void registerYoDoubles(YoRegistry registry, int numberOfYoDoublesToRegister)
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
