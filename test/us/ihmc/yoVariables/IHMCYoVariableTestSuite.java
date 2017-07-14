package us.ihmc.yoVariables;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;
import us.ihmc.yoVariables.dataBuffer.DataBufferEntryTest;
import us.ihmc.yoVariables.dataBuffer.DataBufferTest;
import us.ihmc.yoVariables.dataBuffer.KeyPointsTest;
import us.ihmc.yoVariables.registry.NameSpaceTest;
import us.ihmc.yoVariables.registry.YoVariableRegistryTest;
import us.ihmc.yoVariables.variable.*;

@RunWith(Suite.class)
@Suite.SuiteClasses({
      DataBufferEntryTest.class,
      DataBufferTest.class,
      KeyPointsTest.class,
      NameSpaceTest.class,
      YoVariableRegistryTest.class,
      YoBooleanTest.class,
      YoDoubleTest.class,
      YoEnumTest.class,
      YoIntegerTest.class,
      YoVariableListTest.class,
      YoVariableTest.class,
      YoVariableHolderImplementationTest.class,
      YoVariableHolderImplementationNewTest.class
})
public class IHMCYoVariableTestSuite
{
}
