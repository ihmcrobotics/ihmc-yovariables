package us.ihmc.yoVariables.registry;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.yoVariables.listener.RewoundListener;
import us.ihmc.yoVariables.listener.YoVariableRegistryChangedListener;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.yoVariables.variable.YoVariableList;

import java.io.OutputStream;
import java.io.PrintStream;
import java.util.ArrayList;

import static us.ihmc.robotics.Assert.*;

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
         public void yoVariableWasRegistered(YoVariableRegistry registry, YoVariable<?> variable)
         {            
            lastRegisteredVariable = variable;
         }
         
         public void yoVariableRegistryWasCleared(YoVariableRegistry yoVariableRegistry)
         {        
            lastClearedRegistry = yoVariableRegistry;
         }

         public void yoVariableRegistryWasAdded(YoVariableRegistry addedYoVariableRegistry)
         {          
            lastAddedRegistry = addedYoVariableRegistry;
         }
      };
   }

   private void createAndAddNYoVariables(int numberVariablesToAdd, YoVariableRegistry registry)
   {
      if (numberVariablesToAdd >= 1) new YoDouble("variableOne", registry);
      if (numberVariablesToAdd >= 2) new YoDouble("variableTwo", registry);
      if (numberVariablesToAdd >= 3) new YoDouble("variableThree", registry);
      if (numberVariablesToAdd >= 4) new YoDouble("variableFour", registry);
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

	@Test// timeout=300000,expected = RuntimeException.class
   public void testCantAddChildWithSameName()
   {
      String name = "sameName";
      YoVariableRegistry child1 = new YoVariableRegistry(name);
      YoVariableRegistry child2 = new YoVariableRegistry(name);
      
      testRegistry.addChild(child1);
      testRegistry.addChild(child2);
   }

	@Test// timeout=300000
   public void testGetName()
   {      
      assertEquals("robot", robotRegistry.getName());
      assertEquals("controller", controllerRegistry.getName());
      assertEquals("testRegistry", testRegistry.getName());
   }

	@Test// timeout=300000
   public void testCreateVarList()
   {
      YoVariableList varList = testRegistry.createVarList();
      assertTrue(varList.size() == 4);
      assertTrue(varList.getName() == testRegistry.getNameSpace().getName());
   }

	@Test// timeout=300000
   public void testGetAllVariables()
   {
      ArrayList<YoVariable<?>> allVars = testRegistry.getAllVariablesIncludingDescendants();
      assertTrue(allVars.size() == 4);
   }

	@Test// timeout=300000
   public void testGetNameSpace()
   {
      NameSpace expectedReturn = new NameSpace("robot.controller.testRegistry");
      NameSpace actualReturn = testRegistry.getNameSpace();
      assertEquals("return value", expectedReturn, actualReturn);
   }
   
	@Test// timeout=300000
   public void testGetVariable()
   {
      YoVariable<?> variableOne = testRegistry.getVariable("variableOne");
      YoVariable<?> variableTwo = testRegistry.getVariable("variableTwo");
      YoVariable<?> variableThree = testRegistry.getVariable("variableThree");
      YoVariable<?> variableFour = testRegistry.getVariable("variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      YoVariable<?> doesntExist = testRegistry.getVariable("fooy");
      assertTrue(doesntExist == null);

      variableOne = testRegistry.getVariable("robot.controller.testRegistry.variableOne");
      variableTwo = testRegistry.getVariable("robot.controller.testRegistry.variableTwo");
      variableThree = testRegistry.getVariable("robot.controller.testRegistry.variableThree");
      variableFour = testRegistry.getVariable("robot.controller.testRegistry.variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      variableOne = testRegistry.getVariable("testRegistry.variableOne");
      variableTwo = testRegistry.getVariable("controller.testRegistry.variableTwo");
      variableThree = testRegistry.getVariable("testRegistry.variableThree");
      variableFour = testRegistry.getVariable("controller.testRegistry.variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));


      variableOne = testRegistry.getVariable("robot.controller.variableOne");
      variableTwo = testRegistry.getVariable("robot.testRegistry.variableTwo");
      variableThree = testRegistry.getVariable("bot.controller.testRegistry.variableThree");
      variableFour = testRegistry.getVariable("robot.controller.testRegis.variableFour");

      assertNull(variableOne);
      assertNull(variableTwo);
      assertNull(variableThree);
      assertNull(variableFour);
   }
   
   
	@Test// timeout=300000
   public void testCaseInsensitivityToNameButNotNamespace()
   {
      YoVariable<?> variableOne = testRegistry.getVariable("variableone");
      YoVariable<?> variableTwo = testRegistry.getVariable("variableTWO");
      YoVariable<?> variableThree = testRegistry.getVariable("VAriableThree");
      YoVariable<?> variableFour = testRegistry.getVariable("variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      variableOne = testRegistry.getVariable("robot.controller.testRegistry.variableONE");
      variableTwo = testRegistry.getVariable("robot.controller.testRegistry.variableTwo");
      variableThree = testRegistry.getVariable("robot.controller.testRegistry.variableTHREe");
      variableFour = testRegistry.getVariable("robot.controller.testRegistry.VAriableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      variableOne = testRegistry.getVariable("testRegistry.variableONE");
      variableTwo = testRegistry.getVariable("controller.testRegistry.variableTWO");
      variableThree = testRegistry.getVariable("testRegistry.variableThREE");
      variableFour = testRegistry.getVariable("controller.testRegistry.VAriableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      variableOne = testRegistry.getVariable("Robot.controller.testRegistry.variableOne");
      variableTwo = testRegistry.getVariable("robot.coNtroller.testRegistry.variableTwo");
      variableThree = testRegistry.getVariable("robot.controller.TestRegistry.variableThree");
      variableFour = testRegistry.getVariable("robot.controller.testRegistrY.variableFour");

      assertNull(variableOne);
      assertNull(variableTwo);
      assertNull(variableThree);
      assertNull(variableFour);
   }

	@Test// timeout=300000
   public void testGetVariable1()
   {
      String nameSpace = "robot.controller.testRegistry";

      YoVariable<?> variableOne = testRegistry.getVariable(nameSpace, "variableOne");
      YoVariable<?> variableTwo = testRegistry.getVariable(nameSpace, "variableTwo");
      YoVariable<?> variableThree = testRegistry.getVariable(nameSpace, "variableThree");
      YoVariable<?> variableFour = testRegistry.getVariable(nameSpace, "variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      nameSpace = "controller.testRegistry";

      variableOne = testRegistry.getVariable(nameSpace, "variableOne");
      variableTwo = testRegistry.getVariable(nameSpace, "variableTwo");
      variableThree = testRegistry.getVariable(nameSpace, "variableThree");
      variableFour = testRegistry.getVariable(nameSpace, "variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));

      nameSpace = "testRegistry";

      variableOne = testRegistry.getVariable(nameSpace, "variableOne");
      variableTwo = testRegistry.getVariable(nameSpace, "variableTwo");
      variableThree = testRegistry.getVariable(nameSpace, "variableThree");
      variableFour = testRegistry.getVariable(nameSpace, "variableFour");

      assertTrue(variableOne.getName().equals("variableOne"));
      assertTrue(variableTwo.getName().equals("variableTwo"));
      assertTrue(variableThree.getName().equals("variableThree"));
      assertTrue(variableFour.getName().equals("variableFour"));


      nameSpace = ".testRegistry";

      variableOne = testRegistry.getVariable(nameSpace, "variableOne");
      variableTwo = testRegistry.getVariable(nameSpace, "variableTwo");
      variableThree = testRegistry.getVariable(nameSpace, "variableThree");
      variableFour = testRegistry.getVariable(nameSpace, "variableFour");

      assertNull(variableOne);
      assertNull(variableTwo);
      assertNull(variableThree);
      assertNull(variableFour);

      boolean testPassed = true;
      try
      {
         testRegistry.getVariable(nameSpace, "foo.variableOne");
         testPassed = false;
      }
      catch (RuntimeException e)
      {
      }

      assertTrue(testPassed);
   }

	@Test// timeout=300000
   public void testGetVariables1()
   {
      ArrayList<YoVariable<?>> variables = testRegistry.getVariables("variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("variableTwo");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("variableThree");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("variableFour");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("variable");
      assertTrue(variables.size() == 0);

      variables = testRegistry.getVariables("robot.controller.testRegistry.variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("controller.testRegistry.variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("testRegistry.variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("estRegistry.variableOne");
      assertTrue(variables.size() == 0);

      variables = testRegistry.getVariables("foo.robot.controller.testRegistry.variableOne");
      assertTrue(variables.size() == 0);
   }

	@Test// timeout=300000
   public void testGetVariables2()
   {
      ArrayList<YoVariable<?>> variables = testRegistry.getVariables("robot.controller.testRegistry", "variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("robot.controller.testRegistry", "variableTwo");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("robot.controller.testRegistry", "variableThree");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("robot.controller.testRegistry", "variableFour");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("robot.controller.testRegistry", "variable");
      assertTrue(variables.size() == 0);

      variables = testRegistry.getVariables("controller.testRegistry", "variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("testRegistry", "variableOne");
      assertTrue(variables.size() == 1);

      variables = testRegistry.getVariables("estRegistry", "variableOne");
      assertTrue(variables.size() == 0);

      variables = testRegistry.getVariables("foo.robot.controller.testRegistry", "variableOne");
      assertTrue(variables.size() == 0);

      boolean testPassed = true;
      try
      {
         variables = testRegistry.getVariables("robot.controller.testRegistry", "robot.controller.testRegistry.variableOne");
         testPassed = false;
      }
      catch (Exception e)
      {
      }

      assertTrue(testPassed);

   }

	@Test// timeout=300000
   public void testHasUniqueVariable()
   {
      String name = "";
      boolean expectedReturn = false;
      boolean actualReturn = testRegistry.hasUniqueVariable(name);
      assertEquals("return value", expectedReturn, actualReturn);

      assertTrue(testRegistry.hasUniqueVariable("variableOne"));
      assertFalse(testRegistry.hasUniqueVariable("dontHaveMeVariable"));
      
      assertTrue(testRegistry.hasUniqueVariable("robot.controller.testRegistry", "variableTwo"));
      assertTrue(testRegistry.hasUniqueVariable("controller.testRegistry", "variableTwo"));
      assertFalse(testRegistry.hasUniqueVariable("robot.controller", "variableTwo"));
   }

	@Test// timeout=300000
   public void testHasUniqueVariable1()
   {
      String nameSpace = "";
      String name = "";
      boolean expectedReturn = false;
      boolean actualReturn = testRegistry.hasUniqueVariable(nameSpace, name);
      assertEquals("return value", expectedReturn, actualReturn);

      /** @todo fill in the test code */
   }

	@Test// timeout=300000
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

      assertTrue(testRegistry.hasUniqueVariable("variableFive"));
   }

	@Test// timeout=300000,expected = RuntimeException.class
   public void testCannotRegisterSameVariableName()
   {
      YoDouble variableFiveOnce = new YoDouble("variableFive", null);
      YoDouble variableFiveTwice = new YoDouble("variableFive", null);

      testRegistry.registerVariable(variableFiveOnce);
      assertTrue(testRegistry.hasUniqueVariable("variableFive"));
      testRegistry.registerVariable(variableFiveTwice);
   }

	@Test// timeout=300000
   public void testGetAllVariablesInThisListOnly()
   {
      ArrayList<YoVariable<?>> robotVariablesOnly = robotRegistry.getAllVariablesInThisListOnly();
      ArrayList<YoVariable<?>> controlVariablesOnly = controllerRegistry.getAllVariablesInThisListOnly();
      ArrayList<YoVariable<?>> testRegistryVariablesOnly = testRegistry.getAllVariablesInThisListOnly();
      
      assertEquals(1, robotVariablesOnly.size());
      assertEquals(1, controlVariablesOnly.size());
      assertEquals(4, testRegistryVariablesOnly.size());
   }

	@Test// timeout=300000
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

	@Test// timeout=300000,expected = RuntimeException.class
   public void testDontLetAChildGetAddedToTwoRegistries()
   {
      YoVariableRegistry root1 = new YoVariableRegistry("root1");
      YoVariableRegistry root2 = new YoVariableRegistry("root2");
      
      YoVariableRegistry child = new YoVariableRegistry("child");
      root1.addChild(child);
      root2.addChild(child);
   }

	@Test// timeout=300000
   public void testIllegalName1()
   {
      String illegalName = "foo..foo";
      boolean runtimeExceptionThrown = false;
      try
      {
         testRegistry = new YoVariableRegistry(illegalName);
      }
      catch(RuntimeException e)
      {
         assertTrue(e.getMessage().contains(illegalName));
         runtimeExceptionThrown = true;
      }

      assertTrue(runtimeExceptionThrown);
   }

	@Test// timeout=300000,expected = RuntimeException.class
   public void testIllegalName2()
   {
      testRegistry = new YoVariableRegistry("foo.");
   }

	@Test// timeout=300000,expected = RuntimeException.class
   public void testNoDotsAllowed()
   {
      testRegistry = new YoVariableRegistry("foo.bar");
   }

	@Test// timeout=300000,expected = RuntimeException.class
   public void testIllegalAddChild()
   {
      YoVariableRegistry childOne = new YoVariableRegistry("childOne");
      childOne.addChild(childOne);
   }

	@Test// timeout=300000
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
      
      ArrayList<YoVariable<?>> allVariables = testRegistry.getAllVariablesIncludingDescendants();
      assertEquals(nVarsExpected, allVariables.size());
      YoVariable<?>[] allVariablesArray = testRegistry.getAllVariablesArray();
      assertEquals(nVarsExpected, allVariablesArray.length);

      assertEquals(nVarsChildTwo, childTwo.getAllVariablesIncludingDescendants().size());
   }

	@Test// timeout=300000
   public void testFamilyRelations()
   {
      YoVariableRegistry childOne = new YoVariableRegistry("childOne");
      YoVariableRegistry childTwo = new YoVariableRegistry("childTwo");
      
      testRegistry.addChild(childOne);
      testRegistry.addChild(childTwo);

      assertEquals(childOne.getParent(), testRegistry);
      assertEquals(childTwo.getParent(), testRegistry);

      ArrayList<YoVariableRegistry> children = testRegistry.getChildren();

      int childrenSize = children.size();

      assertEquals(2, childrenSize);
      assertTrue(children.contains(childOne));
      assertTrue(children.contains(childTwo));
   }

	@Test// timeout=300000,expected = RuntimeException.class
   public void testCantAddDuplicateSubnames()
   {
      YoVariableRegistry childOne = new YoVariableRegistry("childOne");
      testRegistry.addChild(childOne);

      YoVariableRegistry grandChildOne = new YoVariableRegistry(childOne.getParent().getNameSpace().getRootName());
      childOne.addChild(grandChildOne);
   }

	@Test// timeout=300000
   public void testNullChild()
   {

      YoVariableRegistry nullChild = null;
      YoVariableRegistry testNullChild = new YoVariableRegistry("TestNullChild");

      testNullChild.addChild(nullChild);
      assertEquals(0, testNullChild.getChildren().size());
   }

	@Test// timeout=300000
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
      
      ArrayList<YoVariable<?>> allRootVariables = root.getAllVariablesIncludingDescendants();
      assertEquals(8, allRootVariables.size());
      
      assertTrue(registry10.hasUniqueVariable("root.registry1.registry10.variable10_A"));
      
      assertEquals(variable10_A, registry10.getVariable("root.registry1.registry10.variable10_A"));
      assertEquals(variable10_A, registry10.getVariable("registry10.variable10_A"));
      assertEquals(variable10_A, registry10.getVariable("variable10_A"));
      
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

      assertEquals(variable10_A, registry1.getVariable("variable10_A"));
      assertEquals(variable10_A, root.getVariable("variable10_A"));
      assertEquals(variable011_A, root.getVariable("variable011_A"));
      assertEquals(variable011_A, registry0.getVariable("variable011_A"));
      assertEquals(variable011_A, registry01.getVariable("variable011_A"));
      assertEquals(variable011_A, registry011.getVariable("variable011_A"));

      assertEquals(repeatedVariable_root, root.getVariable("repeatedVariable"));
      assertEquals(repeatedVariable_registry0, registry0.getVariable("repeatedVariable"));
      assertEquals(repeatedVariable_registry01, registry01.getVariable("repeatedVariable"));
      assertEquals(repeatedVariable_registry010, registry010.getVariable("repeatedVariable"));

      assertEquals(4, root.getVariables("repeatedVariable").size());
   }

	@Test// timeout=300000,expected = RuntimeException.class
   public void testDontAllowRepeatRegistryNames()
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
   }
   
   
	@Test// timeout=300000
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
      
      assertEquals(foo, root.getVariable("foo"));
      assertEquals(bar, root.getVariable("bar"));
      
      
      assertEquals(registry000, root.getOrCreateAndAddRegistry(new NameSpace("root.registry0.registry00.registry000")));
      assertEquals(registry010, root.getOrCreateAndAddRegistry(new NameSpace("root.registry0.registry01.registry010")));
   }

	@Test// timeout=300000
   public void testNullNameSpace()
   {
      YoVariableRegistry root = new YoVariableRegistry("");
      assertEquals(null, root.getNameSpace());

      YoVariableRegistry registry000 = root.getOrCreateAndAddRegistry(new NameSpace("root.registry0.registry00.registry000"));
      NameSpace nameSpaceCheck = registry000.getNameSpace();
      assertEquals(new NameSpace("root.registry0.registry00.registry000"), nameSpaceCheck);
   }

	@Test// timeout=300000,expected = RuntimeException.class
   public void testCantAddAChildWithANullNamespace()
   {
      YoVariableRegistry root = new YoVariableRegistry("root");
      YoVariableRegistry child = new YoVariableRegistry("");
      root.addChild(child);

   }

	@Test// timeout=300000
   public void testCreateVarListIncludingChildren()
   {
      ArrayList<YoVariableList> varLists = robotRegistry.createVarListsIncludingChildren();
      assertEquals(3, varLists.size());
            
      assertContainsListWithNameAndVariables(varLists, "robot", 1);
      assertContainsListWithNameAndVariables(varLists, "robot.controller", 1);
      assertContainsListWithNameAndVariables(varLists, "robot.controller.testRegistry", 4);

   }
   
   private void assertContainsListWithNameAndVariables(ArrayList<YoVariableList> varLists, String name, int numVariables)
   {
      int containsName = 0;
      
      for (YoVariableList varList : varLists)
      {
         if (varList.getName().equals(name))
         {
            containsName++;
            assertEquals(numVariables, varList.getVariables().size());
         }
      }
      
      assertEquals(1, containsName);
   }

	@Test// timeout=300000
   public void testGetAllRegistriesIncludingChildren()
   {
      ArrayList<YoVariableRegistry> registries = robotRegistry.getAllRegistriesIncludingChildren();
      
      assertEquals(3, registries.size());
      assertTrue(registries.contains(robotRegistry));
      assertTrue(registries.contains(controllerRegistry));
      assertTrue(registries.contains(testRegistry));
   }

	@Test// timeout=300000 
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

	@Test// timeout=300000
   public void testLoggingAndSending()
   {
      robotRegistry.setLoggingIncludingDescendants(false);
      robotRegistry.setSendingIncludingDescendants(false);
      
      assertFalse(robotRegistry.isLogged());
      assertFalse(controllerRegistry.isLogged());
      assertFalse(testRegistry.isLogged());
      
      assertFalse(robotRegistry.isSent());
      assertFalse(controllerRegistry.isSent());
      assertFalse(testRegistry.isSent());
      
      controllerRegistry.setLogging(true);
      assertFalse(robotRegistry.isLogged());
      assertTrue(controllerRegistry.isLogged());
      assertFalse(testRegistry.isLogged());
      
      assertFalse(robotRegistry.isSent());
      assertFalse(controllerRegistry.isSent());
      assertFalse(testRegistry.isSent());
      
      controllerRegistry.setSendingIncludingDescendants(true);
      assertFalse(robotRegistry.isLogged());
      assertTrue(controllerRegistry.isLogged());
      assertFalse(testRegistry.isLogged());
      
      assertFalse(robotRegistry.isSent());
      assertTrue(controllerRegistry.isSent());
      assertTrue(testRegistry.isSent());
      
      robotRegistry.setLoggingIncludingDescendants(false);
      robotRegistry.setSendingIncludingDescendants(true);
      
      assertFalse(robotRegistry.isLogged());
      assertFalse(controllerRegistry.isLogged());
      assertFalse(testRegistry.isLogged());
      
      assertTrue(robotRegistry.isSent());
      assertTrue(controllerRegistry.isSent());
      assertTrue(testRegistry.isSent());
   }

	@Test// timeout=300000
   public void testGetNumberOfVariables()
   {
      assertEquals(1, robotRegistry.getNumberOfYoVariables());
      assertEquals(1, controllerRegistry.getNumberOfYoVariables());
      assertEquals(4, testRegistry.getNumberOfYoVariables());
   }

	@Test// timeout=300000
   public void testListenersOne()
   {
      this.robotRegistry.attachYoVariableRegistryChangedListener(listener);
      
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

	@Test// timeout=300000,expected = RuntimeException.class
   public void testThrowExceptionIfAttachListenerToNonRoot()
   {
      this.controllerRegistry.attachYoVariableRegistryChangedListener(listener);
   }

	@Test// timeout=300000
   public void testThrowExceptionIfAddChildRegistryWithAListener()
   {
      YoVariableRegistry childRegistry = new YoVariableRegistry("childToAdd");
      childRegistry.attachYoVariableRegistryChangedListener(listener);
      
      try
      {
         this.controllerRegistry.addChild(childRegistry);
         fail("Should not get here!");
      }
      catch(RuntimeException runtimeException)
      {
      }
   }

   @Test// timeout = 30000
   public void testAreNotEqual()
   {
      assertFalse(robotRegistry.areEqual(null));
      assertFalse(robotRegistry.areEqual(controllerRegistry));

      YoVariableRegistry robotRegistryClone = new YoVariableRegistry("robot");
      assertFalse(robotRegistry.areEqual(robotRegistryClone));

      new YoDouble("robotVariable", robotRegistryClone);
      assertFalse(robotRegistry.areEqual(robotRegistryClone));

      robotRegistryClone.setLogging(true);
      assertFalse(robotRegistry.areEqual(robotRegistryClone));

      robotRegistryClone.setLogging(false);
      robotRegistryClone.setSending(true);
      assertFalse(robotRegistry.areEqual(robotRegistryClone));

      robotRegistryClone.setSending(false);
      robotRegistryClone.setDisallowSending();
      assertFalse(robotRegistry.areEqual(robotRegistryClone));
   }

   @Test// timeout = 30000
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

   @Test// timeout = 30000
   public void testSetDisallowSending()
   {
      assertFalse(robotRegistry.isDisallowSendingSet());
      robotRegistry.setDisallowSending();
      assertTrue(robotRegistry.isDisallowSendingSet());
   }

   @Test// timeout = 30000
   public void testPrintSizeRecursively()
   {
      YoVariableRegistry rootRegistry = new YoVariableRegistry("rootRegistry");

      int numberOfFirstLevelChildRegistries = 2;
      int numberOfFirstLevelYoVariables = 1;
      int numberOfSecondLevelChildRegistries = 1;
      int numberOfSecondLevelYoVariables = 4;
      int numberOfThirdLevelChildRegistries = 1;
      int numberOfThirdLevelYoVariables = 1;

      int totalNumberOfYoVariables = numberOfFirstLevelChildRegistries * (numberOfFirstLevelYoVariables +
                                     numberOfSecondLevelChildRegistries * (numberOfSecondLevelYoVariables +
                                     numberOfThirdLevelChildRegistries * numberOfThirdLevelYoVariables));

      for(int i = 0; i < numberOfFirstLevelChildRegistries; i++)
      {
         YoVariableRegistry firstLevelChild = new YoVariableRegistry("firstLevelChild_" + i);
         registerYoDoubles(firstLevelChild, numberOfFirstLevelYoVariables);
         rootRegistry.addChild(firstLevelChild);

         for(int j = 0; j < numberOfSecondLevelChildRegistries; j++)
         {
            YoVariableRegistry secondLevelChild = new YoVariableRegistry("secondLevelChild_" + j);
            registerYoDoubles(secondLevelChild, numberOfSecondLevelYoVariables);
            firstLevelChild.addChild(secondLevelChild);

            for(int k = 0; k < numberOfThirdLevelChildRegistries; k++)
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
      assertTrue(strings[2-1].contains(String.valueOf(totalNumberOfYoVariables)));
      assertTrue(strings[5-1].contains("firstLevelChild_0.secondLevelChild_0"));
      assertTrue(strings[5-1].contains("Variables: " + numberOfSecondLevelYoVariables));
      assertTrue(strings[5-1].contains("Children: " + numberOfSecondLevelChildRegistries));
      assertTrue(strings[6-1].contains("firstLevelChild_1.secondLevelChild_0"));
      assertTrue(strings[6-1].contains("Variables: " + numberOfSecondLevelYoVariables));
      assertTrue(strings[6-1].contains("Children: " + numberOfSecondLevelChildRegistries));
      assertTrue(strings[7-1].contains("rootRegistry"));
      assertTrue(strings[7-1].contains("Variables: " + 0));
      assertTrue(strings[7-1].contains("Children: " + numberOfFirstLevelChildRegistries));
   }

   private void registerYoDoubles(YoVariableRegistry registry, int numberOfYoDoublesToRegister)
   {
      for(int i = 0; i < numberOfYoDoublesToRegister; i++)
      {
         new YoDouble("yoDouble_" + i, registry);
      }
   }

   @Test// timeout = 30000
   public void testRegisterSimulationRewoundListener()
   {
      RewoundListener rewoundListener = () -> {};

      robotRegistry.registerSimulationRewoundListener(rewoundListener);
      ArrayList<RewoundListener> allSimulationRewoundListeners = robotRegistry.getAllSimulationRewoundListeners();

      assertTrue(allSimulationRewoundListeners.contains(rewoundListener));
   }

   @Test// timeout = 30000
   public void testChangeNameSpace()
   {
      String newNameSpace = "newNameSpace";

      robotRegistry.changeNameSpace(newNameSpace);
      assertTrue(robotRegistry.getNameSpace().getName().equals(newNameSpace));
   }

   @Test// timeout = 30000
   public void testRecursivelyChangingNameSpace()
   {
      String newNameSpace = "newNameSpace";

      assertFalse(robotRegistry.getNameSpace().contains(newNameSpace));
      robotRegistry.getChildren().forEach(registry -> assertFalse(registry.getNameSpace().contains(newNameSpace)));

      NameSpaceRenamer nameSpaceRenamer = nameSpaceString -> newNameSpace;
      robotRegistry.recursivelyChangeNameSpaces(nameSpaceRenamer);

      assertTrue(robotRegistry.getNameSpace().contains(newNameSpace));
      robotRegistry.getChildren().forEach(registry -> assertTrue(registry.getNameSpace().contains(newNameSpace)));
   }

   @Test// timeout = 30000
   public void testClear()
   {
      assertFalse(robotRegistry.getAllVariables().size() == 0);
      assertFalse(robotRegistry.getChildren().size() == 0);

      robotRegistry.clear();

      assertTrue(robotRegistry.getAllVariables().size() == 0);
      assertTrue(robotRegistry.getChildren().size() == 0);
   }

   @Test// timeout = 30000
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

   @Test// timeout = 30000
   public void testGetVariables()
   {
      NameSpace robotNameSpace = new NameSpace("robot");
      ArrayList<YoVariable<?>> robotVariables = robotRegistry.getVariables(robotNameSpace);

      assertTrue(robotVariables.size() == 1);
      assertTrue(robotVariables.contains(robotVariable));
      assertFalse(robotVariables.contains(controlVariable));

      NameSpace controllerNameSpace = new NameSpace("robot.controller");
      ArrayList<YoVariable<?>> controllerVariables = robotRegistry.getVariables(controllerNameSpace);

      assertTrue(controllerVariables.size() == 1);
      assertFalse(controllerVariables.contains(robotVariable));
      assertTrue(controllerVariables.contains(controlVariable));
   }

   @Test// timeout = 30000
   public void testGetMatchingVariables()
   {
      ArrayList<YoVariable<?>> nullVariables = robotRegistry.getMatchingVariables(null, null);
      assertTrue(nullVariables.size() == 0);

      String[] variableNames = new String[]{"robotVariable", "controlVariable"};
      String[] regularExpressions = new String[]{"&controller&"};

      ArrayList<YoVariable<?>> matchingVariablesWithNullRegex = robotRegistry.getMatchingVariables(variableNames, null);
      assertTrue(matchingVariablesWithNullRegex.size() == 2);

      ArrayList<YoVariable<?>> matchingVariablesWithRegex = robotRegistry.getMatchingVariables(null, regularExpressions);
      assertTrue(matchingVariablesWithRegex.size() == 0);
   }

   @Test// timeout = 30000
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
      
      assertEquals(3, a.getAllParameters().size());
      assertEquals(3, aa.getAllParameters().size());
      assertEquals(2, aaa.getAllParameters().size());
      assertEquals(0, ab.getAllParameters().size());
      assertEquals(0, aab.getAllParameters().size());
      
      assertEquals(1, aa.getParametersInThisRegistry().size());
      assertEquals(2, aaa.getParametersInThisRegistry().size());
      assertEquals(0, a.getParametersInThisRegistry().size());
      assertEquals(0, ab.getParametersInThisRegistry().size());
      assertEquals(0, aab.getParametersInThisRegistry().size());
      
      assertTrue(a.getAllParameters().contains(paramater1));
      assertTrue(a.getAllParameters().contains(paramater2));
      assertTrue(a.getAllParameters().contains(paramater3));

      assertTrue(aa.getParametersInThisRegistry().contains(paramater1));
      assertTrue(aaa.getParametersInThisRegistry().contains(paramater2));
      assertTrue(aaa.getParametersInThisRegistry().contains(paramater3));
      
   }
   
   @Test// expected = NullPointerException.class
   public void testCloseAndDispose()
   {
      assertTrue(robotRegistry != null);

      robotRegistry.closeAndDispose();

      assertTrue(robotRegistry.getAllVariables() == null);
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
