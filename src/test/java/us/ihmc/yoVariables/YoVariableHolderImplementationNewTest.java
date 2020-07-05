package us.ihmc.yoVariables;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertNull;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.dataBuffer.YoVariableHolderImplementation;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoVariableHolderImplementationNewTest
{

   public enum EnumYoVariableTestEnums
   {
      ONE, TWO;
   }

   private YoVariableHolderImplementation yoVariableHolderImplementation;
   private ArrayList<YoVariable<?>> testVariables;

   public YoVariableHolderImplementationNewTest()
   {

   }

   @BeforeEach
   public void setUp()
   {
      yoVariableHolderImplementation = new YoVariableHolderImplementation();
      testVariables = new ArrayList<>();
      testVariables.add(new YoDouble("yoDouble", null));
      testVariables.add(new YoBoolean("yoBoolean", null));
      testVariables.add(new YoInteger("yoInteger", null));
      testVariables.add(new YoEnum<>("yoEnum", null, EnumYoVariableTestEnums.class));
   }

   @AfterEach
   public void tearDown()
   {
      yoVariableHolderImplementation = null;
      testVariables = null;
   }

   @Test // timeout=300000
   public void testAddSingleYoVariableToHolderAndGetVariableByName()
   {
      YoDouble yoDoubleFromArrayList = (YoDouble) testVariables.get(0);
      yoVariableHolderImplementation.addVariableToHolder(yoDoubleFromArrayList);
      assertEquals(yoDoubleFromArrayList, yoVariableHolderImplementation.findVariable("yoDouble"));
   }

   @Test // timeout=300000
   public void testAddMultipleYoVariablesToHolderAndGetAllVariables()
   {
      yoVariableHolderImplementation.addVariablesToHolder(testVariables);

      for (YoVariable<?> var : yoVariableHolderImplementation.getVariables())
      {
         assertTrue(testVariables.contains(var));
      }
   }

   @Test // timeout=300000
   public void testGetVariableUsingFullNamespace()
   {
      yoVariableHolderImplementation.addVariablesToHolder(testVariables);
      assertTrue(testVariables.get(0) == yoVariableHolderImplementation.getVariableUsingFullNamespace("yoDouble"));
   }

   //   @Test// timeout=300000
   //   public void testGetVariableUsingFullNamespaceError()
   //   {
   //      yoVariableHolderImplementation.addVariablesToHolder(testVariables);
   //      yoVariableHolderImplementation.getVariableUsingFullNamespace("notPresent");
   //      ByteArrayOutputStream stdErrorContents = new ByteArrayOutputStream();
   //      PrintStream stdErr = new PrintStream(System.err);
   //      System.setErr(new PrintStream(stdErrorContents));allihmc
   //      System.setErr(stdErr);
   //      assertEquals("Warning: " + "notPresent" + " not found. (YoVariableHolderImplementation.getVariable)", stdErrorContents.toString());
   //
   //
   //   }

   @Test // timeout=300000
   public void testGetVariable()
   {
      yoVariableHolderImplementation.addVariablesToHolder(testVariables);
      assertTrue(testVariables.get(0) == yoVariableHolderImplementation.findVariable("yoDouble"));
   }

   @Test // timeout=300000
   public void testGetVariableCaseInsensitive()
   {
      yoVariableHolderImplementation.addVariablesToHolder(testVariables);
      YoVariable<?> variable = yoVariableHolderImplementation.findVariable("YODouble");
      assertTrue(testVariables.get(0) == variable);
   }

   @Test // timeout=300000
   public void testGetVariableWithNameSpace()
   {
      YoRegistry testRegistry;
      testRegistry = new YoRegistry("testRegistry");
      YoDouble yoDoubleWithNameSpace = new YoDouble("yoDoubleWithNameSpace", testRegistry);
      yoVariableHolderImplementation.addVariableToHolder(yoDoubleWithNameSpace);
      assertEquals(yoDoubleWithNameSpace, yoVariableHolderImplementation.findVariable("testRegistry", "yoDoubleWithNameSpace"));
   }

   @Test // timeout=300000
   public void testGetVariableWithNameSpaceCaseInsensitiveExceptNameSpace()
   {
      YoRegistry testRegistry;
      testRegistry = new YoRegistry("testRegistry");
      YoDouble yoDoubleWithNameSpace = new YoDouble("yoDoubleWithNameSpace", testRegistry);
      yoVariableHolderImplementation.addVariableToHolder(yoDoubleWithNameSpace);
      assertEquals(yoDoubleWithNameSpace, yoVariableHolderImplementation.findVariable("testRegistry", "yoDOUBLEWithNameSpace"));
      assertNull(yoVariableHolderImplementation.findVariable("TESTRegistry", "yoDoubleWithNameSpace"));
   }

   @Test // timeout=300000
   public void testHasUniqueVariable()
   {
      yoVariableHolderImplementation.addVariablesToHolder(testVariables);
      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("yoDouble"));
      assertFalse(yoVariableHolderImplementation.hasUniqueVariable("yoDoubleNotPresent"));
      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("yoBoolean"));
      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("yoInteger"));
      assertFalse(yoVariableHolderImplementation.hasUniqueVariable("yoIntegerNotPresent"));
   }

   @Test // timeout=300000
   public void testHasUniqueVariableWithNameSpace()
   {
      YoRegistry testRegistry1;
      YoRegistry testRegistry2;
      testRegistry1 = new YoRegistry("testRegistry1");
      testRegistry2 = new YoRegistry("testRegistry2");
      YoDouble yoDoubleWithNameSpace1 = new YoDouble("yoDoubleWithNameSpace1", testRegistry1);
      YoDouble yoDoubleWithNameSpace2 = new YoDouble("yoDoubleWithNameSpace2", testRegistry2);
      yoVariableHolderImplementation.addVariableToHolder(yoDoubleWithNameSpace1);
      yoVariableHolderImplementation.addVariableToHolder(yoDoubleWithNameSpace2);
      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("testRegistry1", "yoDoubleWithNameSpace1"));
      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("testRegistry2", "yoDoubleWithNameSpace2"));
      assertFalse(yoVariableHolderImplementation.hasUniqueVariable("testRegistry1", "yoDoubleWithNameSpace2"));
      assertFalse(yoVariableHolderImplementation.hasUniqueVariable("testRegistry2", "yoDoubleWithNameSpace1"));
   }

   @Test // timeout=300000
   public void testGetVariablesArrayList() //returns an ArrayList of size 1?
   {
      yoVariableHolderImplementation.addVariablesToHolder(testVariables);
      //      assertEquals(testVariables, yoVariableHolderImplementation.getVariables())
   }

   //testGetVariables(String fullName)

   @Test // timeout=300000
   public void testGetVariablesInNameSpace()
   {
      YoRegistry testRegistry1;
      YoRegistry testRegistry2;
      testRegistry1 = new YoRegistry("testRegistry1");
      testRegistry2 = new YoRegistry("testRegistry2");
      YoDouble yoDoubleWithNameSpace1 = new YoDouble("yoDoubleWithNameSpace1", testRegistry1);
      YoDouble yoDoubleWithNameSpace2 = new YoDouble("yoDoubleWithNameSpace2", testRegistry2);
      YoBoolean yoBooleanWithNameSpace1 = new YoBoolean("yoBooleanWithNameSpace1", testRegistry1);
      YoBoolean yoBooleanWithNameSpace2 = new YoBoolean("yoBooleanWithNameSpace2", testRegistry2);
      YoInteger yoIntegerWithNameSpace1 = new YoInteger("yoIntegerWithNameSpace1", testRegistry1);
      YoInteger yoIntegerWithNameSpace2 = new YoInteger("yoIntegerWithNameSpace2", testRegistry2);
      yoVariableHolderImplementation.addVariableToHolder(yoDoubleWithNameSpace1);
      yoVariableHolderImplementation.addVariableToHolder(yoDoubleWithNameSpace2);
      yoVariableHolderImplementation.addVariableToHolder(yoBooleanWithNameSpace1);
      yoVariableHolderImplementation.addVariableToHolder(yoBooleanWithNameSpace2);
      yoVariableHolderImplementation.addVariableToHolder(yoIntegerWithNameSpace1);
      yoVariableHolderImplementation.addVariableToHolder(yoIntegerWithNameSpace2);

      ArrayList<YoVariable<?>> expectedArrayListFromNameSpaceTestRegistry1 = new ArrayList<>();
      expectedArrayListFromNameSpaceTestRegistry1.add(yoDoubleWithNameSpace1);
      expectedArrayListFromNameSpaceTestRegistry1.add(yoBooleanWithNameSpace1);
      expectedArrayListFromNameSpaceTestRegistry1.add(yoIntegerWithNameSpace1);

      for (int i = 0; i < expectedArrayListFromNameSpaceTestRegistry1.size(); i++)
      {
         assertTrue(expectedArrayListFromNameSpaceTestRegistry1.contains(yoVariableHolderImplementation.getVariables(testRegistry1.getNameSpace()).get(i)));
      }
      //does not return the ArrayList in specified order so contains method was used.
   }

}
