package us.ihmc.yoVariables.registry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoVariableHolderImplementationTest
{
   private YoVariableList yoVariableHolderImplementation = null;

   public YoVariableHolderImplementationTest()
   {
   }

   @BeforeEach
   public void setUp()
   {
      yoVariableHolderImplementation = new YoVariableList("Blop");

      YoRegistry robotRegistry = new YoRegistry("robot");
      YoRegistry robot2Registry = new YoRegistry("robot2");

      YoRegistry registryA = new YoRegistry("registryA");
      YoRegistry registryB = new YoRegistry("registryB");
      YoRegistry registryC = new YoRegistry("registryC");
      robotRegistry.addChild(registryA);
      robotRegistry.addChild(registryB);
      robotRegistry.addChild(registryC);

      YoRegistry registryC2 = new YoRegistry("registryC");
      robot2Registry.addChild(registryC2);

      YoDouble variableOneA = new YoDouble("variableOne", registryA);
      YoDouble variableOneB = new YoDouble("variableOne", registryB);
      YoDouble variableOneC = new YoDouble("variableOne", registryC);
      YoDouble variableOneC2 = new YoDouble("variableOne", registryC2);

      YoDouble variableTwoA = new YoDouble("variableTwo", registryA);
      YoDouble variableTwoB = new YoDouble("variableTwo", registryB);
      YoDouble variableTwoC = new YoDouble("variableTwo", registryC);
      YoDouble variableTwoC2 = new YoDouble("variableTwo", registryC2);

      YoDouble variableThreeA = new YoDouble("variableThree", registryA);
      YoDouble variableThreeB = new YoDouble("variableThree", registryB);
      YoDouble variableThreeC = new YoDouble("variableThree", registryC);
      YoDouble variableThreeC2 = new YoDouble("variableThree", registryC2);

      yoVariableHolderImplementation.add(variableOneA);
      yoVariableHolderImplementation.add(variableOneB);
      yoVariableHolderImplementation.add(variableOneC);
      yoVariableHolderImplementation.add(variableOneC2);

      yoVariableHolderImplementation.add(variableTwoA);
      yoVariableHolderImplementation.add(variableTwoB);
      yoVariableHolderImplementation.add(variableTwoC);
      yoVariableHolderImplementation.add(variableTwoC2);

      yoVariableHolderImplementation.add(variableThreeA);
      yoVariableHolderImplementation.add(variableThreeB);
      yoVariableHolderImplementation.add(variableThreeC);
      yoVariableHolderImplementation.add(variableThreeC2);
   }

   @AfterEach
   public void tearDown()
   {
      yoVariableHolderImplementation = null;
   }

   @Test // timeout=300000
   public void testGetVariable()
   {
      YoVariable variable = yoVariableHolderImplementation.findVariable("robot.registryA.variableOne");
      assertEquals(variable.getName(), "variableOne");

      variable = yoVariableHolderImplementation.findVariable("registryA.variableOne");
      assertEquals(variable.getName(), "variableOne");

      variable = yoVariableHolderImplementation.findVariable("robot.registryA.variableOne");
      assertEquals(variable.getName(), "variableOne");

      variable = yoVariableHolderImplementation.findVariable("istryA.variableOne");
      assertNull(variable);

      variable = yoVariableHolderImplementation.findVariable("robot.registryA.variableTwo");
      assertEquals(variable.getName(), "variableTwo");

   }

   @Test // timeout=300000
   public void testGetVariable1()
   {
      YoVariable variable = yoVariableHolderImplementation.findVariable("robot.registryA", "variableOne");
      assertEquals(variable.getName(), "variableOne");
      assertEquals(variable.getFullNameString(), "robot.registryA.variableOne");

      variable = yoVariableHolderImplementation.findVariable("robot.registryB", "variableOne");
      assertEquals(variable.getName(), "variableOne");
      assertEquals(variable.getFullNameString(), "robot.registryB.variableOne");

      variable = yoVariableHolderImplementation.findVariable("robot.registryC", "variableOne");
      assertEquals(variable.getName(), "variableOne");
      assertEquals(variable.getFullNameString(), "robot.registryC.variableOne");

      variable = yoVariableHolderImplementation.findVariable("registryA", "variableOne");
      assertEquals(variable.getName(), "variableOne");
      assertEquals(variable.getFullNameString(), "robot.registryA.variableOne");

      variable = yoVariableHolderImplementation.findVariable("registryB", "variableTwo");
      assertEquals(variable.getName(), "variableTwo");
      assertEquals(variable.getFullNameString(), "robot.registryB.variableTwo");

      variable = yoVariableHolderImplementation.findVariable("registryC", "variableOne");
      assertEquals(variable.getFullNameString(), "robot.registryC.variableOne");

      variable = yoVariableHolderImplementation.findVariable("registryC", "variableTwo");
      assertEquals(variable.getFullNameString(), "robot.registryC.variableTwo");
   }

   @Test // timeout=300000
   public void testGetVariables()
   {
      YoNamespace namespace = new YoNamespace("robot.registryA");
      List<YoVariable> variables = yoVariableHolderImplementation.findVariables(namespace);
      assertEquals(3, variables.size());

      namespace = new YoNamespace("robot.registryB");
      variables = yoVariableHolderImplementation.findVariables(namespace);
      assertEquals(3, variables.size());

      namespace = new YoNamespace("robot.registryC");
      variables = yoVariableHolderImplementation.findVariables(namespace);
      assertEquals(3, variables.size());

      namespace = new YoNamespace("robot2.registryC");
      variables = yoVariableHolderImplementation.findVariables(namespace);
      assertEquals(3, variables.size());

      namespace = new YoNamespace("robot");
      variables = yoVariableHolderImplementation.findVariables(namespace);
      assertEquals(0, variables.size());

      namespace = new YoNamespace("registryA");
      variables = yoVariableHolderImplementation.findVariables(namespace);
      assertEquals(0, variables.size());

   }

   @Test // timeout=300000
   public void testGetVariables1()
   {
      List<YoVariable> variables = yoVariableHolderImplementation.findVariables("variableOne");
      boolean aFound = false, bFound = false, cFound = false, c2Found = false;

      for (YoVariable variable : variables)
      {
         if (variable.getFullNameString().equals("robot.registryA.variableOne"))
            aFound = true;
         if (variable.getFullNameString().equals("robot.registryB.variableOne"))
            bFound = true;
         if (variable.getFullNameString().equals("robot.registryC.variableOne"))
            cFound = true;
         if (variable.getFullNameString().equals("robot2.registryC.variableOne"))
            c2Found = true;
      }

      assert aFound && bFound && cFound && c2Found;
      assertEquals(4, variables.size());

      variables = yoVariableHolderImplementation.findVariables("variableTwo");
      assertEquals(4, variables.size());

      variables = yoVariableHolderImplementation.findVariables("variableThree");
      assertEquals(4, variables.size());

      variables = yoVariableHolderImplementation.findVariables("var");
      assertEquals(0, variables.size());
   }

   @Test // timeout=300000
   public void testGetVariables2()
   {
      List<YoVariable> variables = yoVariableHolderImplementation.findVariables("robot.registryA", "variableOne");
      assertEquals(1, variables.size());

      variables = yoVariableHolderImplementation.findVariables("robot", "variableOne");
      assertEquals(0, variables.size());

      variables = yoVariableHolderImplementation.findVariables("registryC", "variableOne");
      assertEquals(2, variables.size());
      boolean cFound = false, c2Found = false, testPassed = true;

      for (YoVariable variable : variables)
      {
         if (variable.getFullNameString().equals("robot.registryC.variableOne"))
            cFound = true;
         if (variable.getFullNameString().equals("robot2.registryC.variableOne"))
            c2Found = true;
      }

      assert cFound && c2Found;

      try
      {
         variables = yoVariableHolderImplementation.findVariables("robot", "registryC.variableOne");
         testPassed = false;
      }
      catch (Exception e)
      {
      }

      assert testPassed;

   }

   @Test // timeout=300000
   public void testHasUniqueVariable()
   {

      assertTrue(!yoVariableHolderImplementation.hasUniqueVariable("variableOne"));

      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("robot.registryA.variableOne"));

      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("registryA.variableOne"));

      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("robot.registryA.variableOne"));

      assertTrue(!yoVariableHolderImplementation.hasUniqueVariable("istryA.variableOne"));

      assertTrue(yoVariableHolderImplementation.hasUniqueVariable("robot.registryA.variableTwo"));

      assertTrue(!yoVariableHolderImplementation.hasUniqueVariable("registryC.variableTwo"));
   }

   @Test // timeout=300000
   public void testHasUniqueVariable1()
   {
      assert yoVariableHolderImplementation.hasUniqueVariable("robot.registryA", "variableOne");

      assert yoVariableHolderImplementation.hasUniqueVariable("registryA", "variableOne");

      assert !yoVariableHolderImplementation.hasUniqueVariable("registryC", "variableTwo");

      assert !yoVariableHolderImplementation.hasUniqueVariable("istryA", "variableOne");

      assert yoVariableHolderImplementation.hasUniqueVariable("robot.registryA", "variableTwo");

      assert !yoVariableHolderImplementation.hasUniqueVariable("robot", "variableOne");

      boolean testPassed = true;
      try
      {
         yoVariableHolderImplementation.hasUniqueVariable("robot", "registryC.variableOne");
         testPassed = false;
      }
      catch (Exception e)
      {
      }

      assert testPassed;

   }

}
