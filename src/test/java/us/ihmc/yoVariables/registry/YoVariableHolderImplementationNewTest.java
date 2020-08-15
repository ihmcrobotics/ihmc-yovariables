package us.ihmc.yoVariables.registry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

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
      yoVariableHolderImplementation.add(yoDoubleFromArrayList);
      assertEquals(yoDoubleFromArrayList, yoVariableHolderImplementation.findVariable("yoDouble"));
   }

   @Test // timeout=300000
   public void testAddMultipleYoVariablesToHolderAndGetAllVariables()
   {
      yoVariableHolderImplementation.addAll(testVariables);

      for (YoVariable var : yoVariableHolderImplementation.getVariables())
      {
         assertTrue(testVariables.contains(var));
      }
   }

   @Test // timeout=300000
   public void testGetVariableUsingFullNamespace()
   {
      yoVariableHolderImplementation.addAll(testVariables);
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
      yoVariableHolderImplementation.addAll(testVariables);
      assertTrue(testVariables.get(0) == yoVariableHolderImplementation.findVariable("yoDouble"));
   }

   @Test // timeout=300000
   public void testGetVariableCaseInsensitive()
   {
      yoVariableHolderImplementation.addAll(testVariables);
      YoVariable variable = yoVariableHolderImplementation.findVariable("YODouble");
      assertTrue(testVariables.get(0) == variable);
   }

   @Test // timeout=300000
   public void testGetVariableWithNamespace()
   {
      YoRegistry testRegistry;
      testRegistry = new YoRegistry("testRegistry");
      YoDouble yoDoubleWithNamespace = new YoDouble("yoDoubleWithNamespace", testRegistry);
      yoVariableHolderImplementation.add(yoDoubleWithNamespace);
      assertEquals(yoDoubleWithNamespace, yoVariableHolderImplementation.findVariable("testRegistry", "yoDoubleWithNamespace"));
   }

   @Test // timeout=300000
   public void testGetVariableWithNamespaceCaseInsensitiveExceptNamespace()
   {
      YoRegistry testRegistry;
      testRegistry = new YoRegistry("testRegistry");
      YoDouble yoDoubleWithNamespace = new YoDouble("yoDoubleWithNamespace", testRegistry);
      yoVariableHolderImplementation.add(yoDoubleWithNamespace);
      assertEquals(yoDoubleWithNamespace, yoVariableHolderImplementation.findVariable("testRegistry", "yoDOUBLEWithNamespace"));
      assertNull(yoVariableHolderImplementation.findVariable("TESTRegistry", "yoDoubleWithNamespace"));
   }

   @Test // timeout=300000
   public void testHasUniqueVariable()
   {
      yoVariableHolderImplementation.addAll(testVariables);
      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("yoDouble"));
      assertFalse(yoVariableHolderImplementation.hasUniqueVariable("yoDoubleNotPresent"));
      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("yoBoolean"));
      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("yoInteger"));
      assertFalse(yoVariableHolderImplementation.hasUniqueVariable("yoIntegerNotPresent"));
   }

   @Test // timeout=300000
   public void testHasUniqueVariableWithNamespace()
   {
      YoRegistry testRegistry1;
      YoRegistry testRegistry2;
      testRegistry1 = new YoRegistry("testRegistry1");
      testRegistry2 = new YoRegistry("testRegistry2");
      YoDouble yoDoubleWithNamespace1 = new YoDouble("yoDoubleWithNamespace1", testRegistry1);
      YoDouble yoDoubleWithNamespace2 = new YoDouble("yoDoubleWithNamespace2", testRegistry2);
      yoVariableHolderImplementation.add(yoDoubleWithNamespace1);
      yoVariableHolderImplementation.add(yoDoubleWithNamespace2);
      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("testRegistry1", "yoDoubleWithNamespace1"));
      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("testRegistry2", "yoDoubleWithNamespace2"));
      assertFalse(yoVariableHolderImplementation.hasUniqueVariable("testRegistry1", "yoDoubleWithNamespace2"));
      assertFalse(yoVariableHolderImplementation.hasUniqueVariable("testRegistry2", "yoDoubleWithNamespace1"));
   }

   @Test // timeout=300000
   public void testGetVariablesArrayList() //returns an ArrayList of size 1?
   {
      yoVariableHolderImplementation.addAll(testVariables);
      //      assertEquals(testVariables, yoVariableHolderImplementation.getVariables())
   }

   //testGetVariables(String fullName)

   @Test // timeout=300000
   public void testGetVariablesInNamespace()
   {
      YoRegistry testRegistry1;
      YoRegistry testRegistry2;
      testRegistry1 = new YoRegistry("testRegistry1");
      testRegistry2 = new YoRegistry("testRegistry2");
      YoDouble yoDoubleWithNamespace1 = new YoDouble("yoDoubleWithNamespace1", testRegistry1);
      YoDouble yoDoubleWithNamespace2 = new YoDouble("yoDoubleWithNamespace2", testRegistry2);
      YoBoolean yoBooleanWithNamespace1 = new YoBoolean("yoBooleanWithNamespace1", testRegistry1);
      YoBoolean yoBooleanWithNamespace2 = new YoBoolean("yoBooleanWithNamespace2", testRegistry2);
      YoInteger yoIntegerWithNamespace1 = new YoInteger("yoIntegerWithNamespace1", testRegistry1);
      YoInteger yoIntegerWithNamespace2 = new YoInteger("yoIntegerWithNamespace2", testRegistry2);
      yoVariableHolderImplementation.add(yoDoubleWithNamespace1);
      yoVariableHolderImplementation.add(yoDoubleWithNamespace2);
      yoVariableHolderImplementation.add(yoBooleanWithNamespace1);
      yoVariableHolderImplementation.add(yoBooleanWithNamespace2);
      yoVariableHolderImplementation.add(yoIntegerWithNamespace1);
      yoVariableHolderImplementation.add(yoIntegerWithNamespace2);

      ArrayList<YoVariable> expectedArrayListFromNamespaceTestRegistry1 = new ArrayList<>();
      expectedArrayListFromNamespaceTestRegistry1.add(yoDoubleWithNamespace1);
      expectedArrayListFromNamespaceTestRegistry1.add(yoBooleanWithNamespace1);
      expectedArrayListFromNamespaceTestRegistry1.add(yoIntegerWithNamespace1);

      for (int i = 0; i < expectedArrayListFromNamespaceTestRegistry1.size(); i++)
      {
         assertTrue(expectedArrayListFromNamespaceTestRegistry1.contains(yoVariableHolderImplementation.findVariables(testRegistry1.getNamespace()).get(i)));
      }
      //does not return the ArrayList in specified order so contains method was used.
   }

}
