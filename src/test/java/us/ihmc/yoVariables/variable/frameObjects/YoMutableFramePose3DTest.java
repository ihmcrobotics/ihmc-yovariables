package us.ihmc.yoVariables.variable.frameObjects;

import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.registry.YoVariableRegistry;

class YoMutableFramePose3DTest
{

   @Test
   void testConstructors()
   {
      YoVariableRegistry registry = new YoVariableRegistry("dummy");
      new YoMutableFramePose3D("the", "pose", registry);
   }

}
