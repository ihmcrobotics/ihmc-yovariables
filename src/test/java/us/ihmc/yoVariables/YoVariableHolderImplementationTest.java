package us.ihmc.yoVariables;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertNull;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.dataBuffer.YoVariableHolderImplementation;
import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoVariableHolderImplementationTest
{
   private YoVariableHolderImplementation yoVariableHolderImplementation = null;

   public YoVariableHolderImplementationTest()
   {
   }

   @BeforeEach
   public void setUp()
   {
      yoVariableHolderImplementation = new YoVariableHolderImplementation();

      YoVariableRegistry robotRegistry = new YoVariableRegistry("robot");
      YoVariableRegistry robot2Registry = new YoVariableRegistry("robot2");

      YoVariableRegistry registryA = new YoVariableRegistry("registryA");
      YoVariableRegistry registryB = new YoVariableRegistry("registryB");
      YoVariableRegistry registryC = new YoVariableRegistry("registryC");
      robotRegistry.addChild(registryA);
      robotRegistry.addChild(registryB);
      robotRegistry.addChild(registryC);

      YoVariableRegistry registryC2 = new YoVariableRegistry("registryC");
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

      yoVariableHolderImplementation.addVariableToHolder(variableOneA);
      yoVariableHolderImplementation.addVariableToHolder(variableOneB);
      yoVariableHolderImplementation.addVariableToHolder(variableOneC);
      yoVariableHolderImplementation.addVariableToHolder(variableOneC2);

      yoVariableHolderImplementation.addVariableToHolder(variableTwoA);
      yoVariableHolderImplementation.addVariableToHolder(variableTwoB);
      yoVariableHolderImplementation.addVariableToHolder(variableTwoC);
      yoVariableHolderImplementation.addVariableToHolder(variableTwoC2);

      yoVariableHolderImplementation.addVariableToHolder(variableThreeA);
      yoVariableHolderImplementation.addVariableToHolder(variableThreeB);
      yoVariableHolderImplementation.addVariableToHolder(variableThreeC);
      yoVariableHolderImplementation.addVariableToHolder(variableThreeC2);
   }

   @AfterEach
   public void tearDown()
   {
      yoVariableHolderImplementation = null;
   }

   @Test // timeout=300000
   public void testGetVariable()
   {
      YoVariable<?> variable = yoVariableHolderImplementation.getYoVariable("robot.registryA.variableOne");
      assertEquals(variable.getName(), "variableOne");

      variable = yoVariableHolderImplementation.getYoVariable("registryA.variableOne");
      assertEquals(variable.getName(), "variableOne");

      variable = yoVariableHolderImplementation.getYoVariable("robot.registryA.variableOne");
      assertEquals(variable.getName(), "variableOne");

      variable = yoVariableHolderImplementation.getYoVariable("istryA.variableOne");
      assertNull(variable);

      variable = yoVariableHolderImplementation.getYoVariable("robot.registryA.variableTwo");
      assertEquals(variable.getName(), "variableTwo");

   }

   @Test // timeout=300000
   public void testGetVariable1()
   {
      YoVariable<?> variable = yoVariableHolderImplementation.getYoVariable("robot.registryA", "variableOne");
      assertEquals(variable.getName(), "variableOne");
      assertEquals(variable.getFullNameWithNameSpace(), "robot.registryA.variableOne");

      variable = yoVariableHolderImplementation.getYoVariable("robot.registryB", "variableOne");
      assertEquals(variable.getName(), "variableOne");
      assertEquals(variable.getFullNameWithNameSpace(), "robot.registryB.variableOne");

      variable = yoVariableHolderImplementation.getYoVariable("robot.registryC", "variableOne");
      assertEquals(variable.getName(), "variableOne");
      assertEquals(variable.getFullNameWithNameSpace(), "robot.registryC.variableOne");

      variable = yoVariableHolderImplementation.getYoVariable("registryA", "variableOne");
      assertEquals(variable.getName(), "variableOne");
      assertEquals(variable.getFullNameWithNameSpace(), "robot.registryA.variableOne");

      variable = yoVariableHolderImplementation.getYoVariable("registryB", "variableTwo");
      assertEquals(variable.getName(), "variableTwo");
      assertEquals(variable.getFullNameWithNameSpace(), "robot.registryB.variableTwo");

      boolean testPassed = true;

      try
      {
         variable = yoVariableHolderImplementation.getYoVariable("registryC", "variableOne");
         testPassed = false;
      }
      catch (Exception e)
      {
      }

      assert testPassed;

      try
      {
         variable = yoVariableHolderImplementation.getYoVariable("registryC", "variableTwo");
         testPassed = false;
      }
      catch (Exception e)
      {
      }

      assert testPassed;
   }

   @Test // timeout=300000
   public void testGetVariables()
   {
      NameSpace nameSpace = new NameSpace("robot.registryA");
      List<YoVariable<?>> variables = yoVariableHolderImplementation.getVariables(nameSpace);
      assertEquals(3, variables.size());

      nameSpace = new NameSpace("robot.registryB");
      variables = yoVariableHolderImplementation.getVariables(nameSpace);
      assertEquals(3, variables.size());

      nameSpace = new NameSpace("robot.registryC");
      variables = yoVariableHolderImplementation.getVariables(nameSpace);
      assertEquals(3, variables.size());

      nameSpace = new NameSpace("robot2.registryC");
      variables = yoVariableHolderImplementation.getVariables(nameSpace);
      assertEquals(3, variables.size());

      nameSpace = new NameSpace("robot");
      variables = yoVariableHolderImplementation.getVariables(nameSpace);
      assertEquals(0, variables.size());

      nameSpace = new NameSpace("registryA");
      variables = yoVariableHolderImplementation.getVariables(nameSpace);
      assertEquals(0, variables.size());

   }

   @Test // timeout=300000
   public void testGetVariables1()
   {
      List<YoVariable<?>> variables = yoVariableHolderImplementation.getYoVariables("variableOne");
      boolean aFound = false, bFound = false, cFound = false, c2Found = false;

      for (YoVariable<?> variable : variables)
      {
         if (variable.getFullNameWithNameSpace().equals("robot.registryA.variableOne"))
            aFound = true;
         if (variable.getFullNameWithNameSpace().equals("robot.registryB.variableOne"))
            bFound = true;
         if (variable.getFullNameWithNameSpace().equals("robot.registryC.variableOne"))
            cFound = true;
         if (variable.getFullNameWithNameSpace().equals("robot2.registryC.variableOne"))
            c2Found = true;
      }

      assert aFound && bFound && cFound && c2Found;
      assertEquals(4, variables.size());

      variables = yoVariableHolderImplementation.getYoVariables("variableTwo");
      assertEquals(4, variables.size());

      variables = yoVariableHolderImplementation.getYoVariables("variableThree");
      assertEquals(4, variables.size());

      variables = yoVariableHolderImplementation.getYoVariables("var");
      assertEquals(0, variables.size());
   }

   @Test // timeout=300000
   public void testGetVariables2()
   {
      List<YoVariable<?>> variables = yoVariableHolderImplementation.getYoVariables("robot.registryA", "variableOne");
      assertEquals(1, variables.size());

      variables = yoVariableHolderImplementation.getYoVariables("robot", "variableOne");
      assertEquals(0, variables.size());

      variables = yoVariableHolderImplementation.getYoVariables("registryC", "variableOne");
      assertEquals(2, variables.size());
      boolean cFound = false, c2Found = false, testPassed = true;

      for (YoVariable<?> variable : variables)
      {
         if (variable.getFullNameWithNameSpace().equals("robot.registryC.variableOne"))
            cFound = true;
         if (variable.getFullNameWithNameSpace().equals("robot2.registryC.variableOne"))
            c2Found = true;
      }

      assert cFound && c2Found;

      try
      {
         variables = yoVariableHolderImplementation.getYoVariables("robot", "registryC.variableOne");
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

      assertTrue(!yoVariableHolderImplementation.hasUniqueYoVariable("variableOne"));

      assertTrue(yoVariableHolderImplementation.hasUniqueYoVariable("robot.registryA.variableOne"));

      assertTrue(yoVariableHolderImplementation.hasUniqueYoVariable("registryA.variableOne"));

      assertTrue(yoVariableHolderImplementation.hasUniqueYoVariable("robot.registryA.variableOne"));

      assertTrue(!yoVariableHolderImplementation.hasUniqueYoVariable("istryA.variableOne"));

      assertTrue(yoVariableHolderImplementation.hasUniqueYoVariable("robot.registryA.variableTwo"));

      assertTrue(!yoVariableHolderImplementation.hasUniqueYoVariable("registryC.variableTwo"));
   }

   @Test // timeout=300000
   public void testHasUniqueVariable1()
   {
      assert yoVariableHolderImplementation.hasUniqueYoVariable("robot.registryA", "variableOne");

      assert yoVariableHolderImplementation.hasUniqueYoVariable("registryA", "variableOne");

      assert !yoVariableHolderImplementation.hasUniqueYoVariable("registryC", "variableTwo");

      assert !yoVariableHolderImplementation.hasUniqueYoVariable("istryA", "variableOne");

      assert yoVariableHolderImplementation.hasUniqueYoVariable("robot.registryA", "variableTwo");

      assert !yoVariableHolderImplementation.hasUniqueYoVariable("robot", "variableOne");

      boolean testPassed = true;
      try
      {
         yoVariableHolderImplementation.hasUniqueYoVariable("robot", "registryC.variableOne");
         testPassed = false;
      }
      catch (Exception e)
      {
      }

      assert testPassed;

   }

}
