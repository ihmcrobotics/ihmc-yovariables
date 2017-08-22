package us.ihmc.yoVariables;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;
import us.ihmc.yoVariables.dataBuffer.DataBufferEntryTest;
import us.ihmc.yoVariables.dataBuffer.DataBufferTest;
import us.ihmc.yoVariables.dataBuffer.KeyPointsTest;
import us.ihmc.yoVariables.parameters.AbstractParameterReaderTest;
import us.ihmc.yoVariables.parameters.BooleanParameterTest;
import us.ihmc.yoVariables.parameters.DefaultParameterReaderTest;
import us.ihmc.yoVariables.parameters.DoubleParameterTest;
import us.ihmc.yoVariables.parameters.EnumParameterTest;
import us.ihmc.yoVariables.parameters.IntegerParameterTest;
import us.ihmc.yoVariables.parameters.LongParameterTest;
import us.ihmc.yoVariables.parameters.XMLParameterIOTest;
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
      YoLongTest.class,
      YoVariableListTest.class,
      YoVariableTest.class,
      YoVariableHolderImplementationTest.class,
      YoVariableHolderImplementationNewTest.class,
      AbstractParameterReaderTest.class,
      BooleanParameterTest.class,
      DefaultParameterReaderTest.class,
      DoubleParameterTest.class,
      EnumParameterTest.class,
      IntegerParameterTest.class,
      LongParameterTest.class,
      XMLParameterIOTest.class
})
public class IHMCYoVariableTestSuite
{
}
