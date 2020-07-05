package us.ihmc.yoVariables.registry;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.robotics.Assert.assertEquals;

import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.variable.YoDouble;

public class YoFactoriesTest
{

   @Test // timeout=300000
   public void testGetOrCreateAndAddRegistry()
   {
      YoVariableRegistry root = new YoVariableRegistry("root");

      YoVariableRegistry registry000 = YoFactories.getOrCreateAndAddRegistry(root, new NameSpace("root.registry0.registry00.registry000"));
      NameSpace nameSpaceCheck = registry000.getNameSpace();
      assertEquals(new NameSpace("root.registry0.registry00.registry000"), nameSpaceCheck);

      YoDouble foo = new YoDouble("foo", registry000);
      assertEquals("root.registry0.registry00.registry000.foo", foo.getFullNameWithNameSpace());

      YoVariableRegistry registry010 = YoFactories.getOrCreateAndAddRegistry(root, new NameSpace("root.registry0.registry01.registry010"));
      YoDouble bar = new YoDouble("bar", registry010);
      assertEquals("root.registry0.registry01.registry010.bar", bar.getFullNameWithNameSpace());

      assertEquals(foo, root.findVariable("foo"));
      assertEquals(bar, root.findVariable("bar"));

      assertEquals(registry000, YoFactories.getOrCreateAndAddRegistry(root, new NameSpace("root.registry0.registry00.registry000")));
      assertEquals(registry010, YoFactories.getOrCreateAndAddRegistry(root, new NameSpace("root.registry0.registry01.registry010")));
   }

   @Test // timeout=300000
   public void testNullNameSpace()
   {
      YoVariableRegistry root = new YoVariableRegistry("");
      assertEquals(null, root.getNameSpace());

      YoVariableRegistry registry000 = YoFactories.getOrCreateAndAddRegistry(root, new NameSpace("root.registry0.registry00.registry000"));
      NameSpace nameSpaceCheck = registry000.getNameSpace();
      assertEquals(new NameSpace("root.registry0.registry00.registry000"), nameSpaceCheck);
   }
}
