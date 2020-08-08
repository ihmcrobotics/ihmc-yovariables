package us.ihmc.yoVariables.registry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.tools.YoTools;

public class NameSpaceTest
{
   @BeforeEach
   public void setUp() throws Exception
   {
   }

   @AfterEach
   public void tearDown() throws Exception
   {
   }

   @Test // timeout = 300000
   public void testConstructors()
   {
      new YoNamespace("robot1.controller1.module1");

      try
      {
         new YoNamespace("");
         fail("Shouldn't be able to create a namespace with no name");
      }
      catch (RuntimeException e)
      {
      }

      try
      {
         new YoNamespace("foo.bar.");
         fail();
      }
      catch (RuntimeException e)
      {
      }

      try
      {
         new YoNamespace(".foo.bar");
         fail();
      }
      catch (RuntimeException e)
      {
      }

      try
      {
         new YoNamespace("foo.foo").checkSanity();
         fail();
      }
      catch (RuntimeException e)
      {
      }

      try
      {
         new YoNamespace("foo.bar.foo").checkSanity();
         fail();
      }
      catch (RuntimeException e)
      {
      }

      try
      {
         new YoNamespace("foo..bar");
         fail();
      }
      catch (RuntimeException e)
      {
      }
   }

   @Test // timeout = 300000
   public void testEquals()
   {
      YoNamespace nameSpace1 = new YoNamespace("robot1.controller1.module1");
      YoNamespace nameSpace2 = new YoNamespace("robot1.controller1.module1");
      YoNamespace nameSpace3 = new YoNamespace("robot1.module1");
      YoNamespace nameSpace4 = new YoNamespace("robot1.controller1.mod");
      YoNamespace nameSpace5 = new YoNamespace("bot1.controller1.module1");

      assertTrue(nameSpace1.equals(nameSpace1));
      assertTrue(nameSpace2.equals(nameSpace2));
      assertTrue(nameSpace3.equals(nameSpace3));
      assertTrue(nameSpace4.equals(nameSpace4));
      assertTrue(nameSpace5.equals(nameSpace5));

      assertTrue(nameSpace1.equals(nameSpace2));
      assertTrue(nameSpace2.equals(nameSpace1));
      assertTrue(!nameSpace1.equals(nameSpace3));
      assertTrue(!nameSpace1.equals(nameSpace4));
      assertTrue(!nameSpace1.equals(nameSpace5));
      assertTrue(!nameSpace3.equals(nameSpace1));
      assertTrue(!nameSpace4.equals(nameSpace1));
      assertTrue(!nameSpace5.equals(nameSpace1));

      assertTrue(!nameSpace1.equals("A string"));
      assertTrue(!nameSpace1.equals(null));
   }

   @Test // timeout = 300000
   public void testStartsWith()
   {
      YoNamespace nameSpace = new YoNamespace("robot1.controller1.module1");
      assertTrue(nameSpace.startsWith("robot1"));
      assertTrue(nameSpace.startsWith("robot1.controller1"));
      assertTrue(nameSpace.startsWith("robot1.controller1.module1"));

      assertTrue(!nameSpace.startsWith("robot1.controller1.mod"));
      assertTrue(!nameSpace.startsWith("robot1.controll"));
      assertTrue(!nameSpace.startsWith("rob"));

      assertTrue(!nameSpace.startsWith(".robot1"));
      assertTrue(!nameSpace.startsWith("robot1."));

      assertTrue(!nameSpace.startsWith(""));
   }

   @Test // timeout = 300000
   public void testEndsWith()
   {
      YoNamespace nameSpace = new YoNamespace("robot1.controller1.module1");
      assertTrue(nameSpace.endsWith("module1"));
      assertTrue(nameSpace.endsWith("controller1.module1"));
      assertTrue(nameSpace.endsWith("robot1.controller1.module1"));

      assertTrue(!nameSpace.endsWith("odule1"));
      assertTrue(!nameSpace.endsWith("ontroller1.module1"));
      assertTrue(!nameSpace.endsWith("obot1.controller1.module1"));

      assertTrue(!nameSpace.endsWith("module1."));
      assertTrue(!nameSpace.endsWith(".module1"));

      assertTrue(!nameSpace.endsWith(""));
   }

   @Test // timeout = 300000
   public void testGetShortName()
   {
      YoNamespace nameSpace = new YoNamespace("robot1.controller1.module1");
      assertTrue(nameSpace.getShortName().equals("module1"));
      assertTrue(nameSpace.getName().equals("robot1.controller1.module1"));

      nameSpace = new YoNamespace("module2");
      assertTrue(nameSpace.getName().equals("module2"));
      assertTrue(nameSpace.getShortName().equals("module2"));
   }

   @Test // timeout = 300000
   public void testContains()
   {
      YoNamespace nameSpace = new YoNamespace("robot1.controller1.module1");

      assertTrue(nameSpace.contains("robot1"));
      assertFalse(nameSpace.contains("notMe"));
      assertFalse(nameSpace.contains("notMe.nope.noway.nada.unhunh"));
      assertTrue(nameSpace.contains("robot1.controller1"));
      assertTrue(nameSpace.contains("robot1.controller1.module1"));
      assertFalse(nameSpace.contains("robot1.notMe"));

      assertTrue(!nameSpace.contains("robot1.controller1.mod"));
      assertTrue(!nameSpace.contains("robot1.controll"));
      assertTrue(!nameSpace.contains("rob"));

      assertTrue(nameSpace.contains("module1"));
      assertTrue(nameSpace.contains("controller1.module1"));
      assertTrue(nameSpace.contains("robot1.controller1.module1"));

      assertTrue(!nameSpace.contains("odule1"));
      assertTrue(!nameSpace.contains("ontroller1.module1"));
      assertTrue(!nameSpace.contains("obot1.controller1.module1"));

      assertTrue(nameSpace.contains("controller1"));
      assertTrue(!nameSpace.contains("controller1.mod"));
      assertTrue(!nameSpace.contains("bot1.controller1"));
      assertTrue(!nameSpace.contains(".controller1"));
      assertTrue(!nameSpace.contains("controller1."));

      assertTrue(!nameSpace.contains(""));
   }

   @Test // timeout = 300000
   public void testStripOffFromBeginning()
   {
      YoNamespace nameSpace = new YoNamespace("root.name1.name2");

      YoNamespace nameSpaceToRemove = new YoNamespace("root");
      YoNamespace newNameSpace = nameSpace.removeStart(nameSpaceToRemove);
      assertEquals(new YoNamespace("name1.name2"), newNameSpace);

      nameSpaceToRemove = new YoNamespace("root.name1");
      newNameSpace = nameSpace.removeStart(nameSpaceToRemove);
      assertEquals(new YoNamespace("name2"), newNameSpace);

      nameSpaceToRemove = new YoNamespace("root.name1.name2");
      newNameSpace = nameSpace.removeStart(nameSpaceToRemove);
      assertEquals(null, newNameSpace);

      nameSpaceToRemove = new YoNamespace("name1");
      newNameSpace = nameSpace.removeStart(nameSpaceToRemove);
      assertEquals(null, newNameSpace);
   }

   @Test // timeout = 300000
   public void testStripOffNameSpaceToGetVariableName()
   {
      assertEquals("variable", YoTools.toShortName("root.level1.level2.variable"));
      assertEquals("variable", YoTools.toShortName("root.variable"));
      assertEquals("variable", YoTools.toShortName("variable"));
   }
}
