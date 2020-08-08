package us.ihmc.yoVariables.registry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.tools.YoFactories;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoFactoriesTest
{

   @Test // timeout=300000
   public void testGetOrCreateAndAddRegistry()
   {
      YoRegistry root = new YoRegistry("root");

      YoRegistry registry000 = YoFactories.findOrCreateRegistry(root, new YoNamespace("root.registry0.registry00.registry000"));
      YoNamespace nameSpaceCheck = registry000.getNameSpace();
      assertEquals(new YoNamespace("root.registry0.registry00.registry000"), nameSpaceCheck);

      YoDouble foo = new YoDouble("foo", registry000);
      assertEquals("root.registry0.registry00.registry000.foo", foo.getFullNameString());

      YoRegistry registry010 = YoFactories.findOrCreateRegistry(root, new YoNamespace("root.registry0.registry01.registry010"));
      YoDouble bar = new YoDouble("bar", registry010);
      assertEquals("root.registry0.registry01.registry010.bar", bar.getFullNameString());

      assertEquals(foo, root.findVariable("foo"));
      assertEquals(bar, root.findVariable("bar"));

      assertEquals(registry000, YoFactories.findOrCreateRegistry(root, new YoNamespace("root.registry0.registry00.registry000")));
      assertEquals(registry010, YoFactories.findOrCreateRegistry(root, new YoNamespace("root.registry0.registry01.registry010")));
   }
}
