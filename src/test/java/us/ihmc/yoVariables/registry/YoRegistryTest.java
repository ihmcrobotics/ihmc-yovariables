package us.ihmc.yoVariables.registry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.exceptions.IllegalNameException;
import us.ihmc.yoVariables.exceptions.IllegalOperationException;
import us.ihmc.yoVariables.listener.YoRegistryChangedListener;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoRegistryTest
{
   private static final int N_VARS_IN_ROOT = 4;
   private YoRegistry robotRegistry = null;
   private YoRegistry controllerRegistry = null;
   private YoRegistry testRegistry = null;

   private YoRegistryChangedListener listener = null;

   private YoVariable lastRegisteredVariable = null;
   private YoRegistry lastAddedRegistry = null;
   private YoRegistry lastRemovedRegistry = null;
   private YoRegistry lastClearedRegistry = null;

   private YoDouble robotVariable;
   private YoDouble controlVariable;

   @BeforeEach
   public void setUp()
   {
      robotRegistry = new YoRegistry("robot");
      controllerRegistry = new YoRegistry("controller");
      testRegistry = new YoRegistry("testRegistry");

      robotRegistry.addChild(controllerRegistry);
      controllerRegistry.addChild(testRegistry);

      robotVariable = new YoDouble("robotVariable", robotRegistry);
      controlVariable = new YoDouble("controlVariable", controllerRegistry);

      createAndAddNYoVariables(N_VARS_IN_ROOT, testRegistry);

      listener = new YoRegistryChangedListener()
      {
         @Override
         public void changed(Change change)
         {
            if (change.wasVariableAdded())
               lastRegisteredVariable = change.getTargetVariable();
            if (change.wasRegistryAdded())
               lastAddedRegistry = change.getTargetRegistry();
            if (change.wasRegistryRemoved())
               lastRemovedRegistry = change.getTargetRegistry();
            if (change.wasCleared())
               lastClearedRegistry = change.getSource();
         }
      };
   }

   private void createAndAddNYoVariables(int numberVariablesToAdd, YoRegistry registry)
   {
      if (numberVariablesToAdd >= 1)
         new YoDouble("variableOne", registry);
      if (numberVariablesToAdd >= 2)
         new YoDouble("variableTwo", registry);
      if (numberVariablesToAdd >= 3)
         new YoDouble("variableThree", registry);
      if (numberVariablesToAdd >= 4)
         new YoDouble("variableFour", registry);
   }

   @AfterEach
   public void tearDown()
   {
      robotRegistry = null;
      controllerRegistry = null;
      testRegistry = null;

      listener = null;

      lastRegisteredVariable = null;
      lastAddedRegistry = null;
      lastClearedRegistry = null;
   }

   @Test
   public void testCantAddChildWithSameName()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         String name = "sameName";
         YoRegistry child1 = new YoRegistry(name);
         YoRegistry child2 = new YoRegistry(name);

         testRegistry.addChild(child1);
         testRegistry.addChild(child2);
      });
   }

   @Test
   public void testGetName()
   {
      assertEquals("robot", robotRegistry.getName());
      assertEquals("controller", controllerRegistry.getName());
      assertEquals("testRegistry", testRegistry.getName());
   }

   @Test
   public void testGetAllVariables()
   {
      List<YoVariable> allVars = testRegistry.collectSubtreeVariables();
      assertTrue(allVars.size() == 4);
   }

   @Test
   public void testGetNamespace()
   {
      YoNamespace expectedReturn = new YoNamespace("robot.controller.testRegistry");
      YoNamespace actualReturn = testRegistry.getNamespace();
      assertEquals(expectedReturn, actualReturn, "return value");
   }

   @Test
   public void testGetVariable()
   {
      YoVariable variableOne = testRegistry.findVariable("variableOne");
      YoVariable variableTwo = testRegistry.findVariable("variableTwo");
      YoVariable variableThree = testRegistry.findVariable("variableThree");
      YoVariable variableFour = testRegistry.findVariable("variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      YoVariable doesntExist = testRegistry.findVariable("fooy");
      assertTrue(doesntExist == null);

      variableOne = testRegistry.findVariable("robot.controller.testRegistry.variableOne");
      variableTwo = testRegistry.findVariable("robot.controller.testRegistry.variableTwo");
      variableThree = testRegistry.findVariable("robot.controller.testRegistry.variableThree");
      variableFour = testRegistry.findVariable("robot.controller.testRegistry.variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      variableOne = testRegistry.findVariable("testRegistry.variableOne");
      variableTwo = testRegistry.findVariable("controller.testRegistry.variableTwo");
      variableThree = testRegistry.findVariable("testRegistry.variableThree");
      variableFour = testRegistry.findVariable("controller.testRegistry.variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      variableOne = testRegistry.findVariable("robot.controller.variableOne");
      variableTwo = testRegistry.findVariable("robot.testRegistry.variableTwo");
      variableThree = testRegistry.findVariable("bot.controller.testRegistry.variableThree");
      variableFour = testRegistry.findVariable("robot.controller.testRegis.variableFour");

      assertNull(variableOne);
      assertNull(variableTwo);
      assertNull(variableThree);
      assertNull(variableFour);
   }

   @Test
   public void testCaseInsensitivityToNameButNotNamespace()
   {
      YoVariable variableOne = testRegistry.findVariable("variableone");
      YoVariable variableTwo = testRegistry.findVariable("variableTWO");
      YoVariable variableThree = testRegistry.findVariable("VAriableThree");
      YoVariable variableFour = testRegistry.findVariable("variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      variableOne = testRegistry.findVariable("robot.controller.testRegistry.variableONE");
      variableTwo = testRegistry.findVariable("robot.controller.testRegistry.variableTwo");
      variableThree = testRegistry.findVariable("robot.controller.testRegistry.variableTHREe");
      variableFour = testRegistry.findVariable("robot.controller.testRegistry.VAriableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      variableOne = testRegistry.findVariable("testRegistry.variableONE");
      variableTwo = testRegistry.findVariable("controller.testRegistry.variableTWO");
      variableThree = testRegistry.findVariable("testRegistry.variableThREE");
      variableFour = testRegistry.findVariable("controller.testRegistry.VAriableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      variableOne = testRegistry.findVariable("Robot.controller.testRegistry.variableOne");
      variableTwo = testRegistry.findVariable("robot.coNtroller.testRegistry.variableTwo");
      variableThree = testRegistry.findVariable("robot.controller.TestRegistry.variableThree");
      variableFour = testRegistry.findVariable("robot.controller.testRegistrY.variableFour");

      assertNull(variableOne);
      assertNull(variableTwo);
      assertNull(variableThree);
      assertNull(variableFour);
   }

   @Test
   public void testGetVariable1()
   {
      String namespace = "robot.controller.testRegistry";

      YoVariable variableOne = testRegistry.findVariable(namespace, "variableOne");
      YoVariable variableTwo = testRegistry.findVariable(namespace, "variableTwo");
      YoVariable variableThree = testRegistry.findVariable(namespace, "variableThree");
      YoVariable variableFour = testRegistry.findVariable(namespace, "variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      namespace = "controller.testRegistry";

      variableOne = testRegistry.findVariable(namespace, "variableOne");
      variableTwo = testRegistry.findVariable(namespace, "variableTwo");
      variableThree = testRegistry.findVariable(namespace, "variableThree");
      variableFour = testRegistry.findVariable(namespace, "variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      namespace = "testRegistry";

      variableOne = testRegistry.findVariable(namespace, "variableOne");
      variableTwo = testRegistry.findVariable(namespace, "variableTwo");
      variableThree = testRegistry.findVariable(namespace, "variableThree");
      variableFour = testRegistry.findVariable(namespace, "variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      namespace = ".testRegistry";

      variableOne = testRegistry.findVariable(namespace, "variableOne");
      variableTwo = testRegistry.findVariable(namespace, "variableTwo");
      variableThree = testRegistry.findVariable(namespace, "variableThree");
      variableFour = testRegistry.findVariable(namespace, "variableFour");

      assertNull(variableOne);
      assertNull(variableTwo);
      assertNull(variableThree);
      assertNull(variableFour);

      boolean testPassed = true;
      try
      {
         testRegistry.findVariable(namespace, "foo.variableOne");
         testPassed = false;
      }
      catch (RuntimeException e)
      {
      }

      assertTrue(testPassed);
   }

   @Test
   public void testGetVariables1()
   {
      List<YoVariable> variables = testRegistry.findVariables("variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.findVariables("variableTwo");
      assertTrue(variables.size() == 1);

      variables = testRegistry.findVariables("variableThree");
      assertTrue(variables.size() == 1);

      variables = testRegistry.findVariables("variableFour");
      assertTrue(variables.size() == 1);

      variables = testRegistry.findVariables("variable");
      assertTrue(variables.size() == 0);

      variables = testRegistry.findVariables("robot.controller.testRegistry.variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.findVariables("controller.testRegistry.variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.findVariables("testRegistry.variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.findVariables("estRegistry.variableOne");
      assertTrue(variables.size() == 0);

      variables = testRegistry.findVariables("foo.robot.controller.testRegistry.variableOne");
      assertTrue(variables.size() == 0);
   }

   @Test
   public void testGetVariables2()
   {
      List<YoVariable> variables = testRegistry.findVariables("robot.controller.testRegistry", "variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.findVariables("robot.controller.testRegistry", "variableTwo");
      assertTrue(variables.size() == 1);

      variables = testRegistry.findVariables("robot.controller.testRegistry", "variableThree");
      assertTrue(variables.size() == 1);

      variables = testRegistry.findVariables("robot.controller.testRegistry", "variableFour");
      assertTrue(variables.size() == 1);

      variables = testRegistry.findVariables("robot.controller.testRegistry", "variable");
      assertTrue(variables.size() == 0);

      variables = testRegistry.findVariables("controller.testRegistry", "variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.findVariables("testRegistry", "variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.findVariables("estRegistry", "variableOne");
      assertTrue(variables.size() == 0);

      variables = testRegistry.findVariables("foo.robot.controller.testRegistry", "variableOne");
      assertTrue(variables.size() == 0);

      boolean testPassed = true;
      try
      {
         variables = testRegistry.findVariables("robot.controller.testRegistry", "robot.controller.testRegistry.variableOne");
         testPassed = false;
      }
      catch (Exception e)
      {
      }

      assertTrue(testPassed);

   }

   @Test
   public void testHasUniqueVariable()
   {
      String name = "";
      boolean expectedReturn = false;
      boolean actualReturn = testRegistry.hasUniqueVariable(name);
      assertEquals(expectedReturn, actualReturn, "return value");

      assertTrue(testRegistry.hasUniqueVariable("variableOne"));
      assertFalse(testRegistry.hasUniqueVariable("dontHaveMeVariable"));

      assertTrue(testRegistry.hasUniqueVariable("robot.controller.testRegistry", "variableTwo"));
      assertTrue(testRegistry.hasUniqueVariable("controller.testRegistry", "variableTwo"));
      assertFalse(testRegistry.hasUniqueVariable("robot.controller", "variableTwo"));
   }

   @Test
   public void testHasUniqueVariable1()
   {
      String namespace = "";
      String name = "";
      boolean expectedReturn = false;
      boolean actualReturn = testRegistry.hasUniqueVariable(namespace, name);
      assertEquals(expectedReturn, actualReturn, "return value");

      /** @todo fill in the test code */
   }

   @Test
   public void testRegisterVariable()
   {
      boolean testPassed = true;
      try
      {
         testRegistry.addVariable(null);
         testPassed = false;
      }
      catch (NullPointerException e)
      {
      }

      assertTrue(testPassed);

      testRegistry.addVariable(new YoDouble("variableFive", null));

      assertTrue(testRegistry.hasUniqueVariable("variableFive"));
   }

   @Test
   public void testCannotRegisterSameVariableName()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         YoDouble variableFiveOnce = new YoDouble("variableFive", null);
         YoDouble variableFiveTwice = new YoDouble("variableFive", null);

         testRegistry.addVariable(variableFiveOnce);
         assertTrue(testRegistry.hasUniqueVariable("variableFive"));
         testRegistry.addVariable(variableFiveTwice);
      });
   }

   @Test
   public void testGetYoVariables()
   {
      List<YoVariable> robotVariablesOnly = robotRegistry.getVariables();
      List<YoVariable> controlVariablesOnly = controllerRegistry.getVariables();
      List<YoVariable> testRegistryVariablesOnly = testRegistry.getVariables();

      assertEquals(1, robotVariablesOnly.size());
      assertEquals(1, controlVariablesOnly.size());
      assertEquals(4, testRegistryVariablesOnly.size());
   }

   @Test
   public void testAddChildAndGetParentAndGetChildren()
   {
      assertEquals(robotRegistry.getParent(), null);

      YoRegistry childOne = new YoRegistry("childOne");
      assertEquals(childOne.getParent(), null);

      testRegistry.addChild(childOne);

      boolean testPassed = true;
      try
      {
         YoRegistry childOneRepeat = new YoRegistry("childOne");

         testRegistry.addChild(childOneRepeat);
         testPassed = false;
      }
      catch (Exception e)
      {
      }

      assertTrue(testPassed);
   }

   @Test
   public void testDontLetAChildGetAddedToTwoRegistries()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         YoRegistry root1 = new YoRegistry("root1");
         root1.setRestrictionLevel(YoRegistryRestrictionLevel.RESTRICTED);
         YoRegistry root2 = new YoRegistry("root2");

         YoRegistry child = new YoRegistry("child");
         root1.addChild(child);
         root2.addChild(child);
      });
   }

   @Test
   public void testIllegalName1()
   {
      String illegalName = "foo..foo";
      boolean runtimeExceptionThrown = false;
      try
      {
         testRegistry = new YoRegistry(illegalName);
      }
      catch (RuntimeException e)
      {
         assertTrue(e.getMessage().contains(illegalName));
         runtimeExceptionThrown = true;
      }

      assertTrue(runtimeExceptionThrown);
   }

   @Test
   public void testIllegalName2()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         testRegistry = new YoRegistry("foo.");
      });
   }

   @Test
   public void testNoDotsAllowed()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         testRegistry = new YoRegistry("foo.bar");
      });
   }

   @Test
   public void testIllegalAddChild()
   {
      Assertions.assertThrows(IllegalOperationException.class, () ->
      {
         YoRegistry childOne = new YoRegistry("childOne");
         childOne.addChild(childOne);
      });
   }

   @Test
   public void testGetAllVariablesIncludingDescendants()
   {
      YoRegistry childOne = new YoRegistry("childOne");
      int nVarsChildOne = 3;
      createAndAddNYoVariables(nVarsChildOne, childOne);

      YoRegistry childTwo = new YoRegistry("childTwo");
      int nVarsChildTwo = 2;
      createAndAddNYoVariables(nVarsChildTwo, childTwo);

      testRegistry.addChild(childOne);
      testRegistry.addChild(childTwo);

      int nVarsExpected = nVarsChildOne + nVarsChildTwo + N_VARS_IN_ROOT;

      List<YoVariable> allVariables = testRegistry.collectSubtreeVariables();
      assertEquals(nVarsExpected, allVariables.size());

      assertEquals(nVarsChildTwo, childTwo.collectSubtreeVariables().size());
   }

   @Test
   public void testFamilyRelations()
   {
      YoRegistry childOne = new YoRegistry("childOne");
      YoRegistry childTwo = new YoRegistry("childTwo");

      testRegistry.addChild(childOne);
      testRegistry.addChild(childTwo);

      assertEquals(childOne.getParent(), testRegistry);
      assertEquals(childTwo.getParent(), testRegistry);

      List<YoRegistry> children = testRegistry.getChildren();

      int childrenSize = children.size();

      assertEquals(2, childrenSize);
      assertTrue(children.contains(childOne));
      assertTrue(children.contains(childTwo));
   }

   @Test
   public void testCantAddDuplicateSubnames()
   {
      Assertions.assertThrows(IllegalNameException.class, () ->
      {
         YoRegistry childOne = new YoRegistry("childOne");
         testRegistry.addChild(childOne);

         YoRegistry grandChildOne = new YoRegistry(childOne.getParent().getNamespace().getRootName());
         childOne.addChild(grandChildOne);
      });
   }

   @Test
   public void testNullChild()
   {

      YoRegistry nullChild = null;
      YoRegistry testNullChild = new YoRegistry("TestNullChild");

      testNullChild.addChild(nullChild);
      assertEquals(0, testNullChild.getChildren().size());
   }

   @Test
   public void testRegistryTree()
   {
      YoRegistry root = new YoRegistry("root");

      YoRegistry registry0 = new YoRegistry("registry0");
      assertEquals("registry0", registry0.getNamespace().getName());
      YoRegistry registry1 = new YoRegistry("registry1");
      YoRegistry registry2 = new YoRegistry("registry2");
      root.addChild(registry0);
      assertEquals("root.registry0", registry0.getNamespace().getName());
      assertEquals("registry0", registry0.getNamespace().getShortName());
      root.addChild(registry2);

      YoRegistry registry00 = new YoRegistry("registry00");
      YoRegistry registry01 = new YoRegistry("registry01");
      registry0.addChild(registry00);
      registry0.addChild(registry01);

      YoRegistry registry10 = new YoRegistry("registry10");
      registry1.addChild(registry10);

      YoRegistry registry010 = new YoRegistry("registry010");
      YoRegistry registry011 = new YoRegistry("registry011");
      registry01.addChild(registry010);

      YoDouble variable0_A = new YoDouble("variable0_A", registry0);
      YoDouble variable0_B = new YoDouble("variable0_B", registry0);
      YoDouble variable10_A = new YoDouble("variable10_A", registry10);
      YoDouble variable011_A = new YoDouble("variable011_A", registry011);

      YoDouble repeatedVariable_root = new YoDouble("repeatedVariable", root);
      YoDouble repeatedVariable_registry0 = new YoDouble("repeatedVariable", registry0);
      YoDouble repeatedVariable_registry01 = new YoDouble("repeatedVariable", registry01);
      YoDouble repeatedVariable_registry010 = new YoDouble("repeatedVariable", registry010);

      // Do some of the addChilds out of order to make sure they work correctly when done out of order.
      root.addChild(registry1);
      registry01.addChild(registry011);
      assertEquals("root.registry0.registry01.registry011", registry011.getNamespace().getName());

      assertEquals("root.registry0.variable0_A", variable0_A.getFullNameString());
      assertEquals("root.registry0.variable0_B", variable0_B.getFullNameString());
      assertEquals("root.registry1.registry10.variable10_A", variable10_A.getFullNameString());
      assertEquals("root.registry0.registry01.registry011.variable011_A", variable011_A.getFullNameString());

      List<YoVariable> allRootVariables = root.collectSubtreeVariables();
      assertEquals(8, allRootVariables.size());

      assertTrue(registry10.hasUniqueVariable("root.registry1.registry10.variable10_A"));

      assertEquals(variable10_A, registry10.findVariable("root.registry1.registry10.variable10_A"));
      assertEquals(variable10_A, registry10.findVariable("registry10.variable10_A"));
      assertEquals(variable10_A, registry10.findVariable("variable10_A"));

      assertTrue(root.hasUniqueVariable("root.registry1.registry10.variable10_A"));
      assertTrue(root.hasUniqueVariable("registry1.registry10.variable10_A"));
      assertTrue(root.hasUniqueVariable("registry10.variable10_A"));
      assertTrue(root.hasUniqueVariable("variable10_A"));

      assertFalse(root.hasUniqueVariable("repeatedVariable"));
      assertFalse(registry0.hasUniqueVariable("repeatedVariable"));
      assertFalse(registry01.hasUniqueVariable("repeatedVariable"));
      assertTrue(registry010.hasUniqueVariable("repeatedVariable"));

      assertTrue(root.hasUniqueVariable("registry0.repeatedVariable"));
      assertTrue(root.hasUniqueVariable("registry0", "repeatedVariable"));
      assertFalse(root.hasUniqueVariable("registry0.noWay"));
      assertFalse(root.hasUniqueVariable("noWay.repeatedVariable"));
      assertFalse(root.hasUniqueVariable("noWay", "repeatedVariable"));

      assertEquals(variable10_A, registry1.findVariable("variable10_A"));
      assertEquals(variable10_A, root.findVariable("variable10_A"));
      assertEquals(variable011_A, root.findVariable("variable011_A"));
      assertEquals(variable011_A, registry0.findVariable("variable011_A"));
      assertEquals(variable011_A, registry01.findVariable("variable011_A"));
      assertEquals(variable011_A, registry011.findVariable("variable011_A"));

      assertEquals(repeatedVariable_root, root.findVariable("repeatedVariable"));
      assertEquals(repeatedVariable_registry0, registry0.findVariable("repeatedVariable"));
      assertEquals(repeatedVariable_registry01, registry01.findVariable("repeatedVariable"));
      assertEquals(repeatedVariable_registry010, registry010.findVariable("repeatedVariable"));

      assertEquals(4, root.findVariables("repeatedVariable").size());
   }

   @Test
   public void testDontAllowRepeatRegistryNames()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         YoRegistry root = new YoRegistry("root");

         YoRegistry levelOne = new YoRegistry("levelOne");

         YoRegistry registryOne = new YoRegistry("registryOne");
         YoRegistry registryOneRepeat = new YoRegistry("registryOne");

         //      YoDouble variableOne = new YoDouble("variableOne", registryOne);
         ////      YoDouble variableOneRepeat = new YoDouble("variableOne", registryOneRepeat);
         //      YoDouble variableTwo = new YoDouble("variableTwo", registryOneRepeat);

         root.addChild(levelOne);

         levelOne.addChild(registryOne);
         levelOne.addChild(registryOneRepeat);
      });
   }

   @Test
   public void testCantAddAChildWithANullNamespace()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         YoRegistry root = new YoRegistry("root");
         YoRegistry child = new YoRegistry("");
         root.addChild(child);
      });

   }

   @Test
   public void testGetAllRegistriesIncludingChildren()
   {
      List<YoRegistry> registries = robotRegistry.collectSubtreeRegistries();

      assertEquals(3, registries.size());
      assertTrue(registries.contains(robotRegistry));
      assertTrue(registries.contains(controllerRegistry));
      assertTrue(registries.contains(testRegistry));
   }

   @Test
   public void testGetRegistry()
   {
      assertEquals(robotRegistry, robotRegistry.findRegistry(new YoNamespace("robot")));
      assertEquals(controllerRegistry, robotRegistry.findRegistry(new YoNamespace("robot.controller")));
      assertEquals(testRegistry, robotRegistry.findRegistry(new YoNamespace("robot.controller.testRegistry")));

      assertEquals(controllerRegistry, controllerRegistry.findRegistry(new YoNamespace("robot.controller")));
      assertEquals(testRegistry, controllerRegistry.findRegistry(new YoNamespace("robot.controller.testRegistry")));

      assertEquals(testRegistry, testRegistry.findRegistry(new YoNamespace("robot.controller.testRegistry")));

      assertTrue(testRegistry == robotRegistry.findRegistry(new YoNamespace("testRegistry")));
      assertTrue(testRegistry == robotRegistry.findRegistry(new YoNamespace("controller.testRegistry")));

      assertTrue(robotRegistry != controllerRegistry.findRegistry(new YoNamespace("robot")));

   }

   @Test
   public void testGetNumberOfVariables()
   {
      assertEquals(1, robotRegistry.getNumberOfVariables());
      assertEquals(1, controllerRegistry.getNumberOfVariables());
      assertEquals(4, testRegistry.getNumberOfVariables());
   }

   @Test
   public void testListenersOne()
   {
      robotRegistry.addListener(listener);

      assertNull(lastRegisteredVariable);
      YoDouble addedYoVariable = new YoDouble("addedLater", controllerRegistry);
      assertEquals(addedYoVariable, lastRegisteredVariable);

      assertNull(lastAddedRegistry);
      YoRegistry addedRegistry = new YoRegistry("addedRegistry");
      testRegistry.addChild(addedRegistry);
      assertEquals(addedRegistry, lastAddedRegistry);

      testRegistry.addListener(listener);
      assertNull(lastClearedRegistry);
      assertNull(lastRemovedRegistry);
      testRegistry.clear();
      assertEquals(testRegistry, lastRemovedRegistry);
      assertEquals(testRegistry, lastClearedRegistry);
   }

   @Test
   public void testAreEqual()
   {
      YoRegistry robotRegistryClone = new YoRegistry("robot");
      YoRegistry controllerRegistryClone = new YoRegistry("controller");
      YoRegistry testRegistryClone = new YoRegistry("testRegistry");

      robotRegistryClone.addChild(controllerRegistryClone);
      controllerRegistryClone.addChild(testRegistryClone);

      new YoDouble("robotVariable", robotRegistryClone);
      new YoDouble("controlVariable", controllerRegistryClone);

      createAndAddNYoVariables(N_VARS_IN_ROOT, testRegistryClone);

      assertTrue(robotRegistry.equals(robotRegistryClone));
   }

   @Test
   public void testClear()
   {
      assertFalse(robotRegistry.collectSubtreeVariables().size() == 0);
      assertFalse(robotRegistry.getChildren().size() == 0);

      robotRegistry.clear();

      assertTrue(robotRegistry.collectSubtreeVariables().size() == 0);
      assertTrue(robotRegistry.getChildren().size() == 0);
   }

   @Test
   public void testGetVariables()
   {
      YoNamespace robotNamespace = new YoNamespace("robot");
      List<YoVariable> robotVariables = robotRegistry.findVariables(robotNamespace);

      assertTrue(robotVariables.size() == 1);
      assertTrue(robotVariables.contains(robotVariable));
      assertFalse(robotVariables.contains(controlVariable));

      YoNamespace controllerNamespace = new YoNamespace("robot.controller");
      List<YoVariable> controllerVariables = robotRegistry.findVariables(controllerNamespace);

      assertTrue(controllerVariables.size() == 1);
      assertFalse(controllerVariables.contains(robotVariable));
      assertTrue(controllerVariables.contains(controlVariable));
   }

   @Test
   public void testParameters()
   {
      YoRegistry a = new YoRegistry("a");
      YoRegistry aa = new YoRegistry("aa");
      YoRegistry ab = new YoRegistry("ab");
      YoRegistry aaa = new YoRegistry("aaa");
      YoRegistry aab = new YoRegistry("aab");

      DoubleParameter paramater1 = new DoubleParameter("parameter1", aa);
      DoubleParameter paramater2 = new DoubleParameter("parameter2", aaa);
      DoubleParameter paramater3 = new DoubleParameter("parameter3", aaa);

      new YoDouble("double1", a);
      new YoDouble("double2", aa);
      new YoDouble("double3", ab);
      new YoDouble("double4", aaa);
      new YoDouble("double5", aab);

      a.addChild(aa);
      a.addChild(ab);
      aa.addChild(aaa);
      aa.addChild(aab);

      assertFalse(a.hasParameters());
      assertTrue(a.hasParametersDeep());
      assertTrue(aa.hasParameters());
      assertTrue(aaa.hasParameters());
      assertFalse(ab.hasParameters());
      assertFalse(aab.hasParameters());

      assertEquals(3, a.collectSubtreeParameters().size());
      assertEquals(3, aa.collectSubtreeParameters().size());
      assertEquals(2, aaa.collectSubtreeParameters().size());
      assertEquals(0, ab.collectSubtreeParameters().size());
      assertEquals(0, aab.collectSubtreeParameters().size());

      assertEquals(1, aa.getParameters().size());
      assertEquals(2, aaa.getParameters().size());
      assertEquals(0, a.getParameters().size());
      assertEquals(0, ab.getParameters().size());
      assertEquals(0, aab.getParameters().size());

      assertTrue(a.collectSubtreeParameters().contains(paramater1));
      assertTrue(a.collectSubtreeParameters().contains(paramater2));
      assertTrue(a.collectSubtreeParameters().contains(paramater3));

      assertTrue(aa.getParameters().contains(paramater1));
      assertTrue(aaa.getParameters().contains(paramater2));
      assertTrue(aaa.getParameters().contains(paramater3));

   }
}
