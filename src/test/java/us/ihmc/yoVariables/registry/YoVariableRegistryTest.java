package us.ihmc.yoVariables.registry;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertNull;
import static us.ihmc.robotics.Assert.assertTrue;
import static us.ihmc.robotics.Assert.fail;

import java.io.OutputStream;
import java.io.PrintStream;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.listener.YoVariableRegistryChangedListener;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoVariableRegistryTest
{
   private static final int N_VARS_IN_ROOT = 4;
   private YoVariableRegistry robotRegistry = null;
   private YoVariableRegistry controllerRegistry = null;
   private YoVariableRegistry testRegistry = null;

   private YoVariableRegistryChangedListener listener = null;

   private YoVariable<?> lastRegisteredVariable = null;
   private YoVariableRegistry lastAddedRegistry = null;
   private YoVariableRegistry lastClearedRegistry = null;

   private YoDouble robotVariable;
   private YoDouble controlVariable;

   @BeforeEach
   public void setUp()
   {
      robotRegistry = new YoVariableRegistry("robot");
      controllerRegistry = new YoVariableRegistry("controller");
      testRegistry = new YoVariableRegistry("testRegistry");

      robotRegistry.addChild(controllerRegistry);
      controllerRegistry.addChild(testRegistry);

      //      yoVariableRegistry = new YoVariableRegistry("robot.controller.testRegistry");

      robotVariable = new YoDouble("robotVariable", robotRegistry);
      controlVariable = new YoDouble("controlVariable", controllerRegistry);

      createAndAddNYoVariables(N_VARS_IN_ROOT, testRegistry);

      listener = new YoVariableRegistryChangedListener()
      {
         @Override
         public void yoVariableWasRegistered(YoVariableRegistry registry, YoVariable<?> variable)
         {
            lastRegisteredVariable = variable;
         }

         @Override
         public void yoVariableRegistryWasCleared(YoVariableRegistry yoVariableRegistry)
         {
            lastClearedRegistry = yoVariableRegistry;
         }

         @Override
         public void yoVariableRegistryWasAdded(YoVariableRegistry addedYoVariableRegistry)
         {
            lastAddedRegistry = addedYoVariableRegistry;
         }
      };
   }

   private void createAndAddNYoVariables(int numberVariablesToAdd, YoVariableRegistry registry)
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

   @Test // timeout=300000,expected = RuntimeException.class
   public void testCantAddChildWithSameName()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         String name = "sameName";
         YoVariableRegistry child1 = new YoVariableRegistry(name);
         YoVariableRegistry child2 = new YoVariableRegistry(name);

         testRegistry.addChild(child1);
         testRegistry.addChild(child2);
      });
   }

   @Test // timeout=300000
   public void testGetName()
   {
      assertEquals("robot", robotRegistry.getName());
      assertEquals("controller", controllerRegistry.getName());
      assertEquals("testRegistry", testRegistry.getName());
   }

   @Test // timeout=300000
   public void testGetAllVariables()
   {
      List<YoVariable<?>> allVars = testRegistry.getSubtreeYoVariables();
      assertTrue(allVars.size() == 4);
   }

   @Test // timeout=300000
   public void testGetNameSpace()
   {
      NameSpace expectedReturn = new NameSpace("robot.controller.testRegistry");
      NameSpace actualReturn = testRegistry.getNameSpace();
      assertEquals("return value", expectedReturn, actualReturn);
   }

   @Test // timeout=300000
   public void testGetVariable()
   {
      YoVariable<?> variableOne = testRegistry.getYoVariable("variableOne");
      YoVariable<?> variableTwo = testRegistry.getYoVariable("variableTwo");
      YoVariable<?> variableThree = testRegistry.getYoVariable("variableThree");
      YoVariable<?> variableFour = testRegistry.getYoVariable("variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      YoVariable<?> doesntExist = testRegistry.getYoVariable("fooy");
      assertTrue(doesntExist == null);

      variableOne = testRegistry.getYoVariable("robot.controller.testRegistry.variableOne");
      variableTwo = testRegistry.getYoVariable("robot.controller.testRegistry.variableTwo");
      variableThree = testRegistry.getYoVariable("robot.controller.testRegistry.variableThree");
      variableFour = testRegistry.getYoVariable("robot.controller.testRegistry.variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      variableOne = testRegistry.getYoVariable("testRegistry.variableOne");
      variableTwo = testRegistry.getYoVariable("controller.testRegistry.variableTwo");
      variableThree = testRegistry.getYoVariable("testRegistry.variableThree");
      variableFour = testRegistry.getYoVariable("controller.testRegistry.variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      variableOne = testRegistry.getYoVariable("robot.controller.variableOne");
      variableTwo = testRegistry.getYoVariable("robot.testRegistry.variableTwo");
      variableThree = testRegistry.getYoVariable("bot.controller.testRegistry.variableThree");
      variableFour = testRegistry.getYoVariable("robot.controller.testRegis.variableFour");

      assertNull(variableOne);
      assertNull(variableTwo);
      assertNull(variableThree);
      assertNull(variableFour);
   }

   @Test // timeout=300000
   public void testCaseInsensitivityToNameButNotNamespace()
   {
      YoVariable<?> variableOne = testRegistry.getYoVariable("variableone");
      YoVariable<?> variableTwo = testRegistry.getYoVariable("variableTWO");
      YoVariable<?> variableThree = testRegistry.getYoVariable("VAriableThree");
      YoVariable<?> variableFour = testRegistry.getYoVariable("variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      variableOne = testRegistry.getYoVariable("robot.controller.testRegistry.variableONE");
      variableTwo = testRegistry.getYoVariable("robot.controller.testRegistry.variableTwo");
      variableThree = testRegistry.getYoVariable("robot.controller.testRegistry.variableTHREe");
      variableFour = testRegistry.getYoVariable("robot.controller.testRegistry.VAriableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      variableOne = testRegistry.getYoVariable("testRegistry.variableONE");
      variableTwo = testRegistry.getYoVariable("controller.testRegistry.variableTWO");
      variableThree = testRegistry.getYoVariable("testRegistry.variableThREE");
      variableFour = testRegistry.getYoVariable("controller.testRegistry.VAriableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      variableOne = testRegistry.getYoVariable("Robot.controller.testRegistry.variableOne");
      variableTwo = testRegistry.getYoVariable("robot.coNtroller.testRegistry.variableTwo");
      variableThree = testRegistry.getYoVariable("robot.controller.TestRegistry.variableThree");
      variableFour = testRegistry.getYoVariable("robot.controller.testRegistrY.variableFour");

      assertNull(variableOne);
      assertNull(variableTwo);
      assertNull(variableThree);
      assertNull(variableFour);
   }

   @Test // timeout=300000
   public void testGetVariable1()
   {
      String nameSpace = "robot.controller.testRegistry";

      YoVariable<?> variableOne = testRegistry.getYoVariable(nameSpace, "variableOne");
      YoVariable<?> variableTwo = testRegistry.getYoVariable(nameSpace, "variableTwo");
      YoVariable<?> variableThree = testRegistry.getYoVariable(nameSpace, "variableThree");
      YoVariable<?> variableFour = testRegistry.getYoVariable(nameSpace, "variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      nameSpace = "controller.testRegistry";

      variableOne = testRegistry.getYoVariable(nameSpace, "variableOne");
      variableTwo = testRegistry.getYoVariable(nameSpace, "variableTwo");
      variableThree = testRegistry.getYoVariable(nameSpace, "variableThree");
      variableFour = testRegistry.getYoVariable(nameSpace, "variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      nameSpace = "testRegistry";

      variableOne = testRegistry.getYoVariable(nameSpace, "variableOne");
      variableTwo = testRegistry.getYoVariable(nameSpace, "variableTwo");
      variableThree = testRegistry.getYoVariable(nameSpace, "variableThree");
      variableFour = testRegistry.getYoVariable(nameSpace, "variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      nameSpace = ".testRegistry";

      variableOne = testRegistry.getYoVariable(nameSpace, "variableOne");
      variableTwo = testRegistry.getYoVariable(nameSpace, "variableTwo");
      variableThree = testRegistry.getYoVariable(nameSpace, "variableThree");
      variableFour = testRegistry.getYoVariable(nameSpace, "variableFour");

      assertNull(variableOne);
      assertNull(variableTwo);
      assertNull(variableThree);
      assertNull(variableFour);

      boolean testPassed = true;
      try
      {
         testRegistry.getYoVariable(nameSpace, "foo.variableOne");
         testPassed = false;
      }
      catch (RuntimeException e)
      {
      }

      assertTrue(testPassed);
   }

   @Test // timeout=300000
   public void testGetVariables1()
   {
      List<YoVariable<?>> variables = testRegistry.getYoVariables("variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getYoVariables("variableTwo");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getYoVariables("variableThree");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getYoVariables("variableFour");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getYoVariables("variable");
      assertTrue(variables.size() == 0);

      variables = testRegistry.getYoVariables("robot.controller.testRegistry.variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getYoVariables("controller.testRegistry.variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getYoVariables("testRegistry.variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getYoVariables("estRegistry.variableOne");
      assertTrue(variables.size() == 0);

      variables = testRegistry.getYoVariables("foo.robot.controller.testRegistry.variableOne");
      assertTrue(variables.size() == 0);
   }

   @Test // timeout=300000
   public void testGetVariables2()
   {
      List<YoVariable<?>> variables = testRegistry.getYoVariables("robot.controller.testRegistry", "variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getYoVariables("robot.controller.testRegistry", "variableTwo");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getYoVariables("robot.controller.testRegistry", "variableThree");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getYoVariables("robot.controller.testRegistry", "variableFour");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getYoVariables("robot.controller.testRegistry", "variable");
      assertTrue(variables.size() == 0);

      variables = testRegistry.getYoVariables("controller.testRegistry", "variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getYoVariables("testRegistry", "variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getYoVariables("estRegistry", "variableOne");
      assertTrue(variables.size() == 0);

      variables = testRegistry.getYoVariables("foo.robot.controller.testRegistry", "variableOne");
      assertTrue(variables.size() == 0);

      boolean testPassed = true;
      try
      {
         variables = testRegistry.getYoVariables("robot.controller.testRegistry", "robot.controller.testRegistry.variableOne");
         testPassed = false;
      }
      catch (Exception e)
      {
      }

      assertTrue(testPassed);

   }

   @Test // timeout=300000
   public void testHasUniqueVariable()
   {
      String name = "";
      boolean expectedReturn = false;
      boolean actualReturn = testRegistry.hasUniqueYoVariable(name);
      assertEquals("return value", expectedReturn, actualReturn);

      assertTrue(testRegistry.hasUniqueYoVariable("variableOne"));
      assertFalse(testRegistry.hasUniqueYoVariable("dontHaveMeVariable"));

      assertTrue(testRegistry.hasUniqueYoVariable("robot.controller.testRegistry", "variableTwo"));
      assertTrue(testRegistry.hasUniqueYoVariable("controller.testRegistry", "variableTwo"));
      assertFalse(testRegistry.hasUniqueYoVariable("robot.controller", "variableTwo"));
   }

   @Test // timeout=300000
   public void testHasUniqueVariable1()
   {
      String nameSpace = "";
      String name = "";
      boolean expectedReturn = false;
      boolean actualReturn = testRegistry.hasUniqueYoVariable(nameSpace, name);
      assertEquals("return value", expectedReturn, actualReturn);

      /** @todo fill in the test code */
   }

   @Test // timeout=300000
   public void testRegisterVariable()
   {
      boolean testPassed = true;
      try
      {
         testRegistry.registerVariable(null);
         testPassed = false;
      }
      catch (NullPointerException e)
      {
      }

      assertTrue(testPassed);

      testRegistry.registerVariable(new YoDouble("variableFive", null));

      assertTrue(testRegistry.hasUniqueYoVariable("variableFive"));
   }

   @Test // timeout=300000,expected = RuntimeException.class
   public void testCannotRegisterSameVariableName()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         YoDouble variableFiveOnce = new YoDouble("variableFive", null);
         YoDouble variableFiveTwice = new YoDouble("variableFive", null);

         testRegistry.registerVariable(variableFiveOnce);
         assertTrue(testRegistry.hasUniqueYoVariable("variableFive"));
         testRegistry.registerVariable(variableFiveTwice);
      });
   }

   @Test // timeout=300000
   public void testGetYoVariables()
   {
      List<YoVariable<?>> robotVariablesOnly = robotRegistry.getYoVariables();
      List<YoVariable<?>> controlVariablesOnly = controllerRegistry.getYoVariables();
      List<YoVariable<?>> testRegistryVariablesOnly = testRegistry.getYoVariables();

      assertEquals(1, robotVariablesOnly.size());
      assertEquals(1, controlVariablesOnly.size());
      assertEquals(4, testRegistryVariablesOnly.size());
   }

   @Test // timeout=300000
   public void testAddChildAndGetParentAndGetChildren()
   {
      assertEquals(robotRegistry.getParent(), null);

      YoVariableRegistry childOne = new YoVariableRegistry("childOne");
      assertEquals(childOne.getParent(), null);

      testRegistry.addChild(childOne);

      boolean testPassed = true;
      try
      {
         YoVariableRegistry childOneRepeat = new YoVariableRegistry("childOne");

         testRegistry.addChild(childOneRepeat);
         testPassed = false;
      }
      catch (Exception e)
      {
      }

      assertTrue(testPassed);
   }

   @Test // timeout=300000,expected = RuntimeException.class
   public void testDontLetAChildGetAddedToTwoRegistries()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         YoVariableRegistry root1 = new YoVariableRegistry("root1");
         YoVariableRegistry root2 = new YoVariableRegistry("root2");

         YoVariableRegistry child = new YoVariableRegistry("child");
         root1.addChild(child);
         root2.addChild(child);
      });
   }

   @Test // timeout=300000
   public void testIllegalName1()
   {
      String illegalName = "foo..foo";
      boolean runtimeExceptionThrown = false;
      try
      {
         testRegistry = new YoVariableRegistry(illegalName);
      }
      catch (RuntimeException e)
      {
         assertTrue(e.getMessage().contains(illegalName));
         runtimeExceptionThrown = true;
      }

      assertTrue(runtimeExceptionThrown);
   }

   @Test // timeout=300000,expected = RuntimeException.class
   public void testIllegalName2()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         testRegistry = new YoVariableRegistry("foo.");
      });
   }

   @Test // timeout=300000,expected = RuntimeException.class
   public void testNoDotsAllowed()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         testRegistry = new YoVariableRegistry("foo.bar");
      });
   }

   @Test // timeout=300000,expected = RuntimeException.class
   public void testIllegalAddChild()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         YoVariableRegistry childOne = new YoVariableRegistry("childOne");
         childOne.addChild(childOne);
      });
   }

   @Test // timeout=300000
   public void testGetAllVariablesIncludingDescendants()
   {
      YoVariableRegistry childOne = new YoVariableRegistry("childOne");
      int nVarsChildOne = 3;
      createAndAddNYoVariables(nVarsChildOne, childOne);

      YoVariableRegistry childTwo = new YoVariableRegistry("childTwo");
      int nVarsChildTwo = 2;
      createAndAddNYoVariables(nVarsChildTwo, childTwo);

      testRegistry.addChild(childOne);
      testRegistry.addChild(childTwo);

      int nVarsExpected = nVarsChildOne + nVarsChildTwo + N_VARS_IN_ROOT;

      List<YoVariable<?>> allVariables = testRegistry.getSubtreeYoVariables();
      assertEquals(nVarsExpected, allVariables.size());

      assertEquals(nVarsChildTwo, childTwo.getSubtreeYoVariables().size());
   }

   @Test // timeout=300000
   public void testFamilyRelations()
   {
      YoVariableRegistry childOne = new YoVariableRegistry("childOne");
      YoVariableRegistry childTwo = new YoVariableRegistry("childTwo");

      testRegistry.addChild(childOne);
      testRegistry.addChild(childTwo);

      assertEquals(childOne.getParent(), testRegistry);
      assertEquals(childTwo.getParent(), testRegistry);

      List<YoVariableRegistry> children = testRegistry.getChildren();

      int childrenSize = children.size();

      assertEquals(2, childrenSize);
      assertTrue(children.contains(childOne));
      assertTrue(children.contains(childTwo));
   }

   @Test // timeout=300000,expected = RuntimeException.class
   public void testCantAddDuplicateSubnames()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         YoVariableRegistry childOne = new YoVariableRegistry("childOne");
         testRegistry.addChild(childOne);

         YoVariableRegistry grandChildOne = new YoVariableRegistry(childOne.getParent().getNameSpace().getRootName());
         childOne.addChild(grandChildOne);
      });
   }

   @Test // timeout=300000
   public void testNullChild()
   {

      YoVariableRegistry nullChild = null;
      YoVariableRegistry testNullChild = new YoVariableRegistry("TestNullChild");

      testNullChild.addChild(nullChild);
      assertEquals(0, testNullChild.getChildren().size());
   }

   @Test // timeout=300000
   public void testRegistryTree()
   {
      YoVariableRegistry root = new YoVariableRegistry("root");

      YoVariableRegistry registry0 = new YoVariableRegistry("registry0");
      assertEquals("registry0", registry0.getNameSpace().getName());
      YoVariableRegistry registry1 = new YoVariableRegistry("registry1");
      YoVariableRegistry registry2 = new YoVariableRegistry("registry2");
      root.addChild(registry0);
      assertEquals("root.registry0", registry0.getNameSpace().getName());
      assertEquals("registry0", registry0.getNameSpace().getShortName());
      root.addChild(registry2);

      YoVariableRegistry registry00 = new YoVariableRegistry("registry00");
      YoVariableRegistry registry01 = new YoVariableRegistry("registry01");
      registry0.addChild(registry00);
      registry0.addChild(registry01);

      YoVariableRegistry registry10 = new YoVariableRegistry("registry10");
      registry1.addChild(registry10);

      YoVariableRegistry registry010 = new YoVariableRegistry("registry010");
      YoVariableRegistry registry011 = new YoVariableRegistry("registry011");
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
      assertEquals("root.registry0.registry01.registry011", registry011.getNameSpace().getName());

      assertEquals("root.registry0.variable0_A", variable0_A.getFullNameWithNameSpace());
      assertEquals("root.registry0.variable0_B", variable0_B.getFullNameWithNameSpace());
      assertEquals("root.registry1.registry10.variable10_A", variable10_A.getFullNameWithNameSpace());
      assertEquals("root.registry0.registry01.registry011.variable011_A", variable011_A.getFullNameWithNameSpace());

      List<YoVariable<?>> allRootVariables = root.getSubtreeYoVariables();
      assertEquals(8, allRootVariables.size());

      assertTrue(registry10.hasUniqueYoVariable("root.registry1.registry10.variable10_A"));

      assertEquals(variable10_A, registry10.getYoVariable("root.registry1.registry10.variable10_A"));
      assertEquals(variable10_A, registry10.getYoVariable("registry10.variable10_A"));
      assertEquals(variable10_A, registry10.getYoVariable("variable10_A"));

      assertTrue(root.hasUniqueYoVariable("root.registry1.registry10.variable10_A"));
      assertTrue(root.hasUniqueYoVariable("registry1.registry10.variable10_A"));
      assertTrue(root.hasUniqueYoVariable("registry10.variable10_A"));
      assertTrue(root.hasUniqueYoVariable("variable10_A"));

      assertFalse(root.hasUniqueYoVariable("repeatedVariable"));
      assertFalse(registry0.hasUniqueYoVariable("repeatedVariable"));
      assertFalse(registry01.hasUniqueYoVariable("repeatedVariable"));
      assertTrue(registry010.hasUniqueYoVariable("repeatedVariable"));

      assertTrue(root.hasUniqueYoVariable("registry0.repeatedVariable"));
      assertTrue(root.hasUniqueYoVariable("registry0", "repeatedVariable"));
      assertFalse(root.hasUniqueYoVariable("registry0.noWay"));
      assertFalse(root.hasUniqueYoVariable("noWay.repeatedVariable"));
      assertFalse(root.hasUniqueYoVariable("noWay", "repeatedVariable"));

      assertEquals(variable10_A, registry1.getYoVariable("variable10_A"));
      assertEquals(variable10_A, root.getYoVariable("variable10_A"));
      assertEquals(variable011_A, root.getYoVariable("variable011_A"));
      assertEquals(variable011_A, registry0.getYoVariable("variable011_A"));
      assertEquals(variable011_A, registry01.getYoVariable("variable011_A"));
      assertEquals(variable011_A, registry011.getYoVariable("variable011_A"));

      assertEquals(repeatedVariable_root, root.getYoVariable("repeatedVariable"));
      assertEquals(repeatedVariable_registry0, registry0.getYoVariable("repeatedVariable"));
      assertEquals(repeatedVariable_registry01, registry01.getYoVariable("repeatedVariable"));
      assertEquals(repeatedVariable_registry010, registry010.getYoVariable("repeatedVariable"));

      assertEquals(4, root.getYoVariables("repeatedVariable").size());
   }

   @Test // timeout=300000,expected = RuntimeException.class
   public void testDontAllowRepeatRegistryNames()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         YoVariableRegistry root = new YoVariableRegistry("root");

         YoVariableRegistry levelOne = new YoVariableRegistry("levelOne");

         YoVariableRegistry registryOne = new YoVariableRegistry("registryOne");
         YoVariableRegistry registryOneRepeat = new YoVariableRegistry("registryOne");

         //      YoDouble variableOne = new YoDouble("variableOne", registryOne);
         ////      YoDouble variableOneRepeat = new YoDouble("variableOne", registryOneRepeat);
         //      YoDouble variableTwo = new YoDouble("variableTwo", registryOneRepeat);

         root.addChild(levelOne);

         levelOne.addChild(registryOne);
         levelOne.addChild(registryOneRepeat);
      });
   }

   @Test // timeout=300000
   public void testGetOrCreateAndAddRegistry()
   {
      YoVariableRegistry root = new YoVariableRegistry("root");

      YoVariableRegistry registry000 = root.getOrCreateAndAddRegistry(new NameSpace("root.registry0.registry00.registry000"));
      NameSpace nameSpaceCheck = registry000.getNameSpace();
      assertEquals(new NameSpace("root.registry0.registry00.registry000"), nameSpaceCheck);

      YoDouble foo = new YoDouble("foo", registry000);
      assertEquals("root.registry0.registry00.registry000.foo", foo.getFullNameWithNameSpace());

      YoVariableRegistry registry010 = root.getOrCreateAndAddRegistry(new NameSpace("root.registry0.registry01.registry010"));
      YoDouble bar = new YoDouble("bar", registry010);
      assertEquals("root.registry0.registry01.registry010.bar", bar.getFullNameWithNameSpace());

      assertEquals(foo, root.getYoVariable("foo"));
      assertEquals(bar, root.getYoVariable("bar"));

      assertEquals(registry000, root.getOrCreateAndAddRegistry(new NameSpace("root.registry0.registry00.registry000")));
      assertEquals(registry010, root.getOrCreateAndAddRegistry(new NameSpace("root.registry0.registry01.registry010")));
   }

   @Test // timeout=300000
   public void testNullNameSpace()
   {
      YoVariableRegistry root = new YoVariableRegistry("");
      assertEquals(null, root.getNameSpace());

      YoVariableRegistry registry000 = root.getOrCreateAndAddRegistry(new NameSpace("root.registry0.registry00.registry000"));
      NameSpace nameSpaceCheck = registry000.getNameSpace();
      assertEquals(new NameSpace("root.registry0.registry00.registry000"), nameSpaceCheck);
   }

   @Test // timeout=300000,expected = RuntimeException.class
   public void testCantAddAChildWithANullNamespace()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         YoVariableRegistry root = new YoVariableRegistry("root");
         YoVariableRegistry child = new YoVariableRegistry("");
         root.addChild(child);
      });

   }

   @Test // timeout=300000
   public void testGetAllRegistriesIncludingChildren()
   {
      List<YoVariableRegistry> registries = robotRegistry.getSubtreeYoVariableRegistries();

      assertEquals(3, registries.size());
      assertTrue(registries.contains(robotRegistry));
      assertTrue(registries.contains(controllerRegistry));
      assertTrue(registries.contains(testRegistry));
   }

   @Test // timeout=300000
   public void testGetRegistry()
   {
      assertEquals(robotRegistry, robotRegistry.getRegistry(new NameSpace("robot")));
      assertEquals(controllerRegistry, robotRegistry.getRegistry(new NameSpace("robot.controller")));
      assertEquals(testRegistry, robotRegistry.getRegistry(new NameSpace("robot.controller.testRegistry")));

      assertEquals(controllerRegistry, controllerRegistry.getRegistry(new NameSpace("robot.controller")));
      assertEquals(testRegistry, controllerRegistry.getRegistry(new NameSpace("robot.controller.testRegistry")));

      assertEquals(testRegistry, testRegistry.getRegistry(new NameSpace("robot.controller.testRegistry")));

      assertTrue(testRegistry != robotRegistry.getRegistry(new NameSpace("testRegistry")));
      assertTrue(testRegistry != robotRegistry.getRegistry(new NameSpace("controller.testRegistry")));

      assertTrue(robotRegistry != controllerRegistry.getRegistry(new NameSpace("robot")));

   }

   @Test // timeout=300000
   public void testGetNumberOfVariables()
   {
      assertEquals(1, robotRegistry.getNumberOfYoVariables());
      assertEquals(1, controllerRegistry.getNumberOfYoVariables());
      assertEquals(4, testRegistry.getNumberOfYoVariables());
   }

   @Test // timeout=300000
   public void testListenersOne()
   {
      robotRegistry.attachYoVariableRegistryChangedListener(listener);

      assertNull(lastRegisteredVariable);
      YoDouble addedYoVariable = new YoDouble("addedLater", controllerRegistry);
      assertEquals(addedYoVariable, lastRegisteredVariable);

      assertNull(lastAddedRegistry);
      YoVariableRegistry addedRegistry = new YoVariableRegistry("addedRegistry");
      testRegistry.addChild(addedRegistry);
      assertEquals(addedRegistry, lastAddedRegistry);

      assertNull(lastClearedRegistry);
      testRegistry.clear();
      assertEquals(testRegistry, lastClearedRegistry);
   }

   @Test // timeout=300000,expected = RuntimeException.class
   public void testThrowExceptionIfAttachListenerToNonRoot()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         controllerRegistry.attachYoVariableRegistryChangedListener(listener);
      });
   }

   @Test // timeout=300000
   public void testThrowExceptionIfAddChildRegistryWithAListener()
   {
      YoVariableRegistry childRegistry = new YoVariableRegistry("childToAdd");
      childRegistry.attachYoVariableRegistryChangedListener(listener);

      try
      {
         controllerRegistry.addChild(childRegistry);
         fail("Should not get here!");
      }
      catch (RuntimeException runtimeException)
      {
      }
   }

   @Test // timeout = 30000
   public void testAreEqual()
   {
      YoVariableRegistry robotRegistryClone = new YoVariableRegistry("robot");
      YoVariableRegistry controllerRegistryClone = new YoVariableRegistry("controller");
      YoVariableRegistry testRegistryClone = new YoVariableRegistry("testRegistry");

      robotRegistryClone.addChild(controllerRegistryClone);
      controllerRegistryClone.addChild(testRegistryClone);

      new YoDouble("robotVariable", robotRegistryClone);
      new YoDouble("controlVariable", controllerRegistryClone);

      createAndAddNYoVariables(N_VARS_IN_ROOT, testRegistryClone);

      assertTrue(robotRegistry.areEqual(robotRegistryClone));
   }

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

      YoVariableRegistry.printSizeRecursively(minimumVariablesToPrint, minimumChildrenToPrint, rootRegistry);

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

   private void registerYoDoubles(YoVariableRegistry registry, int numberOfYoDoublesToRegister)
   {
      for (int i = 0; i < numberOfYoDoublesToRegister; i++)
      {
         new YoDouble("yoDouble_" + i, registry);
      }
   }

   @Test // timeout = 30000
   public void testClear()
   {
      assertFalse(robotRegistry.getSubtreeYoVariables().size() == 0);
      assertFalse(robotRegistry.getChildren().size() == 0);

      robotRegistry.clear();

      assertTrue(robotRegistry.getSubtreeYoVariables().size() == 0);
      assertTrue(robotRegistry.getChildren().size() == 0);
   }

   @Test // timeout = 30000
   public void testPrintAllVariablesIncludingDescendants()
   {
      Interceptor interceptor = new Interceptor(System.out);

      robotRegistry.printAllVariablesIncludingDescendants(interceptor);
      String[] buffer = interceptor.getBuffer();

      assertTrue(buffer.length != 0);
      assertTrue(buffer[0].equals("robot.robotVariable"));
      assertTrue(buffer[1].equals("robot.controller.controlVariable"));
      assertTrue(buffer[2].equals("robot.controller.testRegistry.variableOne"));
      assertTrue(buffer[3].equals("robot.controller.testRegistry.variableTwo"));
      assertTrue(buffer[4].equals("robot.controller.testRegistry.variableThree"));
      assertTrue(buffer[5].equals("robot.controller.testRegistry.variableFour"));
   }

   @Test // timeout = 30000
   public void testGetVariables()
   {
      NameSpace robotNameSpace = new NameSpace("robot");
      List<YoVariable<?>> robotVariables = robotRegistry.getVariables(robotNameSpace);

      assertTrue(robotVariables.size() == 1);
      assertTrue(robotVariables.contains(robotVariable));
      assertFalse(robotVariables.contains(controlVariable));

      NameSpace controllerNameSpace = new NameSpace("robot.controller");
      List<YoVariable<?>> controllerVariables = robotRegistry.getVariables(controllerNameSpace);

      assertTrue(controllerVariables.size() == 1);
      assertFalse(controllerVariables.contains(robotVariable));
      assertTrue(controllerVariables.contains(controlVariable));
   }

   @Test // timeout = 30000
   public void testGetMatchingVariables()
   {
      List<YoVariable<?>> nullVariables = robotRegistry.getMatchingVariables(null, null);
      assertTrue(nullVariables.size() == 0);

      String[] variableNames = new String[] {"robotVariable", "controlVariable"};
      String[] regularExpressions = new String[] {"&controller&"};

      List<YoVariable<?>> matchingVariablesWithNullRegex = robotRegistry.getMatchingVariables(variableNames, null);
      assertTrue(matchingVariablesWithNullRegex.size() == 2);

      List<YoVariable<?>> matchingVariablesWithRegex = robotRegistry.getMatchingVariables(null, regularExpressions);
      assertTrue(matchingVariablesWithRegex.size() == 0);
   }

   @Test // timeout = 30000
   public void testParameters()
   {
      YoVariableRegistry a = new YoVariableRegistry("a");
      YoVariableRegistry aa = new YoVariableRegistry("aa");
      YoVariableRegistry ab = new YoVariableRegistry("ab");
      YoVariableRegistry aaa = new YoVariableRegistry("aaa");
      YoVariableRegistry aab = new YoVariableRegistry("aab");

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

      assertTrue(a.getIfRegistryOrChildrenHaveParameters());
      assertTrue(aa.getIfRegistryOrChildrenHaveParameters());
      assertTrue(aaa.getIfRegistryOrChildrenHaveParameters());
      assertFalse(ab.getIfRegistryOrChildrenHaveParameters());
      assertFalse(aab.getIfRegistryOrChildrenHaveParameters());

      assertEquals(3, a.getSubtreeYoParameters().size());
      assertEquals(3, aa.getSubtreeYoParameters().size());
      assertEquals(2, aaa.getSubtreeYoParameters().size());
      assertEquals(0, ab.getSubtreeYoParameters().size());
      assertEquals(0, aab.getSubtreeYoParameters().size());

      assertEquals(1, aa.getYoParameters().size());
      assertEquals(2, aaa.getYoParameters().size());
      assertEquals(0, a.getYoParameters().size());
      assertEquals(0, ab.getYoParameters().size());
      assertEquals(0, aab.getYoParameters().size());

      assertTrue(a.getSubtreeYoParameters().contains(paramater1));
      assertTrue(a.getSubtreeYoParameters().contains(paramater2));
      assertTrue(a.getSubtreeYoParameters().contains(paramater3));

      assertTrue(aa.getYoParameters().contains(paramater1));
      assertTrue(aaa.getYoParameters().contains(paramater2));
      assertTrue(aaa.getYoParameters().contains(paramater3));

   }

   @Test // expected = NullPointerException.class
   public void testCloseAndDispose()
   {
      Assertions.assertThrows(NullPointerException.class, () ->
      {
         assertTrue(robotRegistry != null);

         robotRegistry.closeAndDispose();

         assertTrue(robotRegistry.getSubtreeYoVariables() == null);
      });
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

   //   public static void main(String[] args)
   //   {
   //      MutationTestFacilitator.facilitateMutationTestForClass(YoVariableRegistry.class, YoVariableRegistryTest.class);
   //   }
}
