package us.ihmc.yoVariables.variable.frameObjects;

import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.registry.YoRegistry;

class YoMutableFramePose3DTest
{

   @Test
   void testConstructors()
   {
      YoRegistry registry = new YoRegistry("dummy");
      new YoMutableFramePose3D("the", "pose", registry);
   }

}
