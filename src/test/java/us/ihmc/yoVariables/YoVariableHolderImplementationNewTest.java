package us.ihmc.yoVariables;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.registry.YoVariableList;
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

   private YoVariableList yoVariableHolderImplementation;
   private ArrayList<YoVariable> testVariables;

   public YoVariableHolderImplementationNewTest()
   {

   }

   @BeforeEach
   public void setUp()
   {
      yoVariableHolderImplementation = new YoVariableList("Blop");
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
      yoVariableHolderImplementation.addVariable(yoDoubleFromArrayList);
      assertEquals(yoDoubleFromArrayList, yoVariableHolderImplementation.findVariable("yoDouble"));
   }

   @Test // timeout=300000
   public void testAddMultipleYoVariablesToHolderAndGetAllVariables()
   {
      yoVariableHolderImplementation.addVariables(testVariables);

      for (YoVariable var : yoVariableHolderImplementation.getVariables())
      {
         assertTrue(testVariables.contains(var));
      }
   }

   @Test // timeout=300000
   public void testGetVariableUsingFullNamespace()
   {
      yoVariableHolderImplementation.addVariables(testVariables);
      assertTrue(testVariables.get(0) == yoVariableHolderImplementation.findVariable("yoDouble"));
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
      yoVariableHolderImplementation.addVariables(testVariables);
      assertTrue(testVariables.get(0) == yoVariableHolderImplementation.findVariable("yoDouble"));
   }

   @Test // timeout=300000
   public void testGetVariableCaseInsensitive()
   {
      yoVariableHolderImplementation.addVariables(testVariables);
      YoVariable variable = yoVariableHolderImplementation.findVariable("YODouble");
      assertTrue(testVariables.get(0) == variable);
   }

   @Test // timeout=300000
   public void testGetVariableWithNameSpace()
   {
      YoRegistry testRegistry;
      testRegistry = new YoRegistry("testRegistry");
      YoDouble yoDoubleWithNameSpace = new YoDouble("yoDoubleWithNameSpace", testRegistry);
      yoVariableHolderImplementation.addVariable(yoDoubleWithNameSpace);
      assertEquals(yoDoubleWithNameSpace, yoVariableHolderImplementation.findVariable("testRegistry", "yoDoubleWithNameSpace"));
   }

   @Test // timeout=300000
   public void testGetVariableWithNameSpaceCaseInsensitiveExceptNameSpace()
   {
      YoRegistry testRegistry;
      testRegistry = new YoRegistry("testRegistry");
      YoDouble yoDoubleWithNameSpace = new YoDouble("yoDoubleWithNameSpace", testRegistry);
      yoVariableHolderImplementation.addVariable(yoDoubleWithNameSpace);
      assertEquals(yoDoubleWithNameSpace, yoVariableHolderImplementation.findVariable("testRegistry", "yoDOUBLEWithNameSpace"));
      assertNull(yoVariableHolderImplementation.findVariable("TESTRegistry", "yoDoubleWithNameSpace"));
   }

   @Test // timeout=300000
   public void testHasUniqueVariable()
   {
      yoVariableHolderImplementation.addVariables(testVariables);
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
      yoVariableHolderImplementation.addVariable(yoDoubleWithNameSpace1);
      yoVariableHolderImplementation.addVariable(yoDoubleWithNameSpace2);
      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("testRegistry1", "yoDoubleWithNameSpace1"));
      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("testRegistry2", "yoDoubleWithNameSpace2"));
      assertFalse(yoVariableHolderImplementation.hasUniqueVariable("testRegistry1", "yoDoubleWithNameSpace2"));
      assertFalse(yoVariableHolderImplementation.hasUniqueVariable("testRegistry2", "yoDoubleWithNameSpace1"));
   }

   @Test // timeout=300000
   public void testGetVariablesArrayList() //returns an ArrayList of size 1?
   {
      yoVariableHolderImplementation.addVariables(testVariables);
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
      yoVariableHolderImplementation.addVariable(yoDoubleWithNameSpace1);
      yoVariableHolderImplementation.addVariable(yoDoubleWithNameSpace2);
      yoVariableHolderImplementation.addVariable(yoBooleanWithNameSpace1);
      yoVariableHolderImplementation.addVariable(yoBooleanWithNameSpace2);
      yoVariableHolderImplementation.addVariable(yoIntegerWithNameSpace1);
      yoVariableHolderImplementation.addVariable(yoIntegerWithNameSpace2);

      ArrayList<YoVariable> expectedArrayListFromNameSpaceTestRegistry1 = new ArrayList<>();
      expectedArrayListFromNameSpaceTestRegistry1.add(yoDoubleWithNameSpace1);
      expectedArrayListFromNameSpaceTestRegistry1.add(yoBooleanWithNameSpace1);
      expectedArrayListFromNameSpaceTestRegistry1.add(yoIntegerWithNameSpace1);

      for (int i = 0; i < expectedArrayListFromNameSpaceTestRegistry1.size(); i++)
      {
         assertTrue(expectedArrayListFromNameSpaceTestRegistry1.contains(yoVariableHolderImplementation.findVariables(testRegistry1.getNameSpace()).get(i)));
      }
      //does not return the ArrayList in specified order so contains method was used.
   }

}
