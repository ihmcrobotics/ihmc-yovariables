package us.ihmc.yoVariables.registry;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.tools.YoTools;

public class YoNamespaceTest
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
      YoNamespace namespace1 = new YoNamespace("robot1.controller1.module1");
      YoNamespace namespace2 = new YoNamespace("robot1.controller1.module1");
      YoNamespace namespace3 = new YoNamespace("robot1.module1");
      YoNamespace namespace4 = new YoNamespace("robot1.controller1.mod");
      YoNamespace namespace5 = new YoNamespace("bot1.controller1.module1");

      assertTrue(namespace1.equals(namespace1));
      assertTrue(namespace2.equals(namespace2));
      assertTrue(namespace3.equals(namespace3));
      assertTrue(namespace4.equals(namespace4));
      assertTrue(namespace5.equals(namespace5));

      assertTrue(namespace1.equals(namespace2));
      assertTrue(namespace2.equals(namespace1));
      assertTrue(!namespace1.equals(namespace3));
      assertTrue(!namespace1.equals(namespace4));
      assertTrue(!namespace1.equals(namespace5));
      assertTrue(!namespace3.equals(namespace1));
      assertTrue(!namespace4.equals(namespace1));
      assertTrue(!namespace5.equals(namespace1));

      assertTrue(!namespace1.equals("A string"));
      assertTrue(!namespace1.equals(null));
   }

   @Test // timeout = 300000
   public void testStartsWith()
   {
      YoNamespace namespace = new YoNamespace("robot1.controller1.module1");
      assertTrue(namespace.startsWith("robot1"));
      assertTrue(namespace.startsWith("robot1.controller1"));
      assertTrue(namespace.startsWith("robot1.controller1.module1"));

      assertTrue(!namespace.startsWith("robot1.controller1.mod"));
      assertTrue(!namespace.startsWith("robot1.controll"));
      assertTrue(!namespace.startsWith("rob"));

      assertTrue(!namespace.startsWith(".robot1"));
      assertTrue(!namespace.startsWith("robot1."));

      assertTrue(!namespace.startsWith(""));
   }

   @Test // timeout = 300000
   public void testEndsWith()
   {
      YoNamespace namespace = new YoNamespace("robot1.controller1.module1");
      assertTrue(namespace.endsWith("module1"));
      assertTrue(namespace.endsWith("controller1.module1"));
      assertTrue(namespace.endsWith("robot1.controller1.module1"));

      assertTrue(!namespace.endsWith("odule1"));
      assertTrue(!namespace.endsWith("ontroller1.module1"));
      assertTrue(!namespace.endsWith("obot1.controller1.module1"));

      assertTrue(!namespace.endsWith("module1."));
      assertTrue(!namespace.endsWith(".module1"));

      assertTrue(!namespace.endsWith(""));
   }

   @Test // timeout = 300000
   public void testGetShortName()
   {
      YoNamespace namespace = new YoNamespace("robot1.controller1.module1");
      assertTrue(namespace.getShortName().equals("module1"));
      assertTrue(namespace.getName().equals("robot1.controller1.module1"));

      namespace = new YoNamespace("module2");
      assertTrue(namespace.getName().equals("module2"));
      assertTrue(namespace.getShortName().equals("module2"));
   }

   @Test // timeout = 300000
   public void testContains()
   {
      YoNamespace namespace = new YoNamespace("robot1.controller1.module1");

      assertTrue(namespace.contains("robot1"));
      assertFalse(namespace.contains("notMe"));
      assertFalse(namespace.contains("notMe.nope.noway.nada.unhunh"));
      assertTrue(namespace.contains("robot1.controller1"));
      assertTrue(namespace.contains("robot1.controller1.module1"));
      assertFalse(namespace.contains("robot1.notMe"));

      assertTrue(!namespace.contains("robot1.controller1.mod"));
      assertTrue(!namespace.contains("robot1.controll"));
      assertTrue(!namespace.contains("rob"));

      assertTrue(namespace.contains("module1"));
      assertTrue(namespace.contains("controller1.module1"));
      assertTrue(namespace.contains("robot1.controller1.module1"));

      assertTrue(!namespace.contains("odule1"));
      assertTrue(!namespace.contains("ontroller1.module1"));
      assertTrue(!namespace.contains("obot1.controller1.module1"));

      assertTrue(namespace.contains("controller1"));
      assertTrue(!namespace.contains("controller1.mod"));
      assertTrue(!namespace.contains("bot1.controller1"));
      assertTrue(!namespace.contains(".controller1"));
      assertTrue(!namespace.contains("controller1."));

      assertTrue(!namespace.contains(""));
   }

   @Test // timeout = 300000
   public void testStripOffFromBeginning()
   {
      YoNamespace namespace = new YoNamespace("root.name1.name2");

      YoNamespace namespaceToRemove = new YoNamespace("root");
      YoNamespace newNamespace = namespace.removeStart(namespaceToRemove);
      assertEquals(new YoNamespace("name1.name2"), newNamespace);

      namespaceToRemove = new YoNamespace("root.name1");
      newNamespace = namespace.removeStart(namespaceToRemove);
      assertEquals(new YoNamespace("name2"), newNamespace);

      namespaceToRemove = new YoNamespace("root.name1.name2");
      newNamespace = namespace.removeStart(namespaceToRemove);
      assertEquals(null, newNamespace);

      namespaceToRemove = new YoNamespace("name1");
      newNamespace = namespace.removeStart(namespaceToRemove);
      assertEquals(null, newNamespace);
   }

   @Test // timeout = 300000
   public void testStripOffNamespaceToGetVariableName()
   {
      assertEquals("variable", YoTools.toShortName("root.level1.level2.variable"));
      assertEquals("variable", YoTools.toShortName("root.variable"));
      assertEquals("variable", YoTools.toShortName("variable"));
   }
}
