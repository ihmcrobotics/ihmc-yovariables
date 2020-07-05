package us.ihmc.yoVariables.dataBuffer;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.listener.RewoundListener;
import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

public class DataBufferTest
{
   private final int testBufferSize = 100;

   private enum EnumYoVariableTestEnums
   {
      ONE, TWO;
   }

   private YoEnum<EnumYoVariableTestEnums> yoEnum;
   private YoDouble yoDouble;
   private YoBoolean yoBoolean;
   private YoInteger yoInteger;
   private YoVariableRegistry registry;
   private DataBuffer dataBuffer = new DataBuffer(testBufferSize);

   private YoDouble a, b, c;
   private DataBufferEntry aBuffer, bBuffer, cBuffer;

   @BeforeEach
   public void setUp()
   {
      registry = new YoVariableRegistry("testRegistry");
      yoDouble = new YoDouble("yoDouble", registry);
      yoBoolean = new YoBoolean("yoBoolean", registry);
      yoInteger = new YoInteger("yoInteger", registry);
      yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class);

      a = new YoDouble("a_arm", registry);
      b = new YoDouble("b_arm", registry);
      c = new YoDouble("c_arm", registry);

      aBuffer = new DataBufferEntry(a, testBufferSize);
      bBuffer = new DataBufferEntry(b, testBufferSize);
      cBuffer = new DataBufferEntry(c, testBufferSize);
   }

   @AfterEach
   public void tearDown()
   {
      yoDouble = null;
      registry = null;
   }

   @Test // timeout = 300000
   public void testGetBufferSize()
   {
      int testBufferSize = dataBuffer.getBufferSize();
      int expectedBufferSize = testBufferSize;
      assertTrue(expectedBufferSize == testBufferSize);
   }

   @Test // timeout = 300000
   public void testGetMaxBufferSize()
   {
      int expectedMaxBufferSize = 16384;
      int testMaxBufferSize = dataBuffer.getMaxBufferSize();
      assertTrue(expectedMaxBufferSize == testMaxBufferSize);
   }

   @Test // timeout = 300000
   public void testGetAndSetWrapBuffer()
   {
      dataBuffer.setWrapBuffer(false);
      boolean testBoolean = dataBuffer.getWrapBuffer();
      assertFalse(testBoolean);
      dataBuffer.setWrapBuffer(true);
      testBoolean = dataBuffer.getWrapBuffer();
      assertTrue(testBoolean);
   }

   @Test // timeout = 300000
   public void testAddAndGetEntry()
   {
      DataBufferEntry doubleDataBufferEntryTest = new DataBufferEntry(yoDouble, testBufferSize);
      DataBufferEntry booleanDataBufferEntryTest = new DataBufferEntry(yoBoolean, testBufferSize);
      DataBufferEntry integerDataBufferEntryTest = new DataBufferEntry(yoInteger, testBufferSize);
      DataBufferEntry enumDataBufferEntryTest = new DataBufferEntry(yoEnum, testBufferSize);
      dataBuffer.addEntry(doubleDataBufferEntryTest);
      dataBuffer.addEntry(booleanDataBufferEntryTest);
      dataBuffer.addEntry(integerDataBufferEntryTest);
      dataBuffer.addEntry(enumDataBufferEntryTest);

      DataBufferEntry testEntryReceivedViaString = dataBuffer.getEntry("yoDouble");
      DataBufferEntry testEntryReceivedViaVariableName = dataBuffer.getEntry(yoDouble);
      assertEquals(doubleDataBufferEntryTest, testEntryReceivedViaString);
      assertEquals(doubleDataBufferEntryTest, testEntryReceivedViaVariableName);

      testEntryReceivedViaString = dataBuffer.getEntry("yoBoolean");
      testEntryReceivedViaVariableName = dataBuffer.getEntry(yoBoolean);
      assertEquals(booleanDataBufferEntryTest, testEntryReceivedViaString);
      assertEquals(booleanDataBufferEntryTest, testEntryReceivedViaVariableName);

      testEntryReceivedViaString = dataBuffer.getEntry("yoInteger");
      testEntryReceivedViaVariableName = dataBuffer.getEntry(yoInteger);
      assertEquals(integerDataBufferEntryTest, testEntryReceivedViaString);
      assertEquals(integerDataBufferEntryTest, testEntryReceivedViaVariableName);

      testEntryReceivedViaString = dataBuffer.getEntry("yoEnum");
      testEntryReceivedViaVariableName = dataBuffer.getEntry(yoEnum);
      assertEquals(enumDataBufferEntryTest, testEntryReceivedViaString);
      assertEquals(enumDataBufferEntryTest, testEntryReceivedViaVariableName);

   }

   @Test // timeout = 300000
   public void testAddNewEntry()
   {
      dataBuffer.addVariable(yoDouble, testBufferSize);
      dataBuffer.addVariable(yoBoolean, testBufferSize);
      dataBuffer.addVariable(yoInteger, testBufferSize);
      dataBuffer.addVariable(yoEnum, testBufferSize);

      DataBufferEntry doubleDataBufferEntryTest = new DataBufferEntry(yoDouble, testBufferSize);
      DataBufferEntry booleanDataBufferEntryTest = new DataBufferEntry(yoBoolean, testBufferSize);
      DataBufferEntry integerDataBufferEntryTest = new DataBufferEntry(yoInteger, testBufferSize);
      DataBufferEntry enumDataBufferEntryTest = new DataBufferEntry(yoEnum, testBufferSize);

      assertTrue(doubleDataBufferEntryTest.getVariable() == dataBuffer.getEntry(yoDouble).getVariable());
      assertTrue(booleanDataBufferEntryTest.getVariable() == dataBuffer.getEntry(yoBoolean).getVariable());
      assertTrue(integerDataBufferEntryTest.getVariable() == dataBuffer.getEntry(yoInteger).getVariable());
      assertTrue(enumDataBufferEntryTest.getVariable() == dataBuffer.getEntry(yoEnum).getVariable());
   }

   @Test // timeout = 300000
   public void testAddVariable()
   {
      dataBuffer.addVariable(yoDouble);
      dataBuffer.addVariable(yoBoolean);
      dataBuffer.addVariable(yoInteger);
      dataBuffer.addVariable(yoEnum);

      assertTrue(yoDouble == dataBuffer.getEntry(yoDouble).getVariable());
      assertTrue(yoBoolean == dataBuffer.getEntry(yoBoolean).getVariable());
      assertTrue(yoInteger == dataBuffer.getEntry(yoInteger).getVariable());
      assertTrue(yoEnum == dataBuffer.getEntry(yoEnum).getVariable());

   }

   @Test // timeout = 300000
   public void testAddVariableWithArrayList()
   {
      ArrayList<YoVariable<?>> arrayListToBeAdded = new ArrayList<>();
      arrayListToBeAdded.add(yoDouble);
      arrayListToBeAdded.add(yoBoolean);
      arrayListToBeAdded.add(yoInteger);
      arrayListToBeAdded.add(yoEnum);

      dataBuffer.addVariables(arrayListToBeAdded);

      assertTrue(yoDouble == dataBuffer.getEntry(yoDouble).getVariable());
      assertTrue(yoBoolean == dataBuffer.getEntry(yoBoolean).getVariable());
      assertTrue(yoInteger == dataBuffer.getEntry(yoInteger).getVariable());
      assertTrue(yoEnum == dataBuffer.getEntry(yoEnum).getVariable());

   }

   //add dataBuffer listener?

   @Test // timeout = 300000
   public void testGetVariablesThatContain()
   {
      YoDouble yoVariable123456789 = new YoDouble("123456789", registry);
      YoDouble yoVariable12345678 = new YoDouble("12345678", registry);
      YoDouble yoVariable1234567 = new YoDouble("1234567", registry);
      YoDouble yoVariable123456 = new YoDouble("123456", registry);
      YoDouble yoVariable12345 = new YoDouble("12345", registry);
      YoDouble yoVariable1234 = new YoDouble("1234", registry);
      YoDouble yoVariable123 = new YoDouble("123", registry);
      YoDouble yoVariable12 = new YoDouble("12", registry);
      YoDouble yoVariable1 = new YoDouble("1", registry);

      ArrayList<YoVariable<?>> currentlyMatched = new ArrayList<>();

      currentlyMatched.add(yoVariable123456789);
      currentlyMatched.add(yoVariable12345678);
      currentlyMatched.add(yoVariable1234567);
      currentlyMatched.add(yoVariable123456);
      currentlyMatched.add(yoVariable12345);
      currentlyMatched.add(yoVariable1234);
      currentlyMatched.add(yoVariable123);
      currentlyMatched.add(yoVariable12);
      currentlyMatched.add(yoVariable1);

      assertTrue(1 == dataBuffer.getVariablesThatContain("123456789", true, currentlyMatched).size());
      assertTrue(2 == dataBuffer.getVariablesThatContain("12345678", true, currentlyMatched).size());
      assertTrue(3 == dataBuffer.getVariablesThatContain("1234567", true, currentlyMatched).size());
      assertTrue(4 == dataBuffer.getVariablesThatContain("123456", true, currentlyMatched).size());
      assertTrue(5 == dataBuffer.getVariablesThatContain("12345", true, currentlyMatched).size());
      assertTrue(6 == dataBuffer.getVariablesThatContain("1234", true, currentlyMatched).size());
      assertTrue(7 == dataBuffer.getVariablesThatContain("123", true, currentlyMatched).size());
      assertTrue(8 == dataBuffer.getVariablesThatContain("12", true, currentlyMatched).size());
      assertTrue(9 == dataBuffer.getVariablesThatContain("1", true, currentlyMatched).size());
      assertTrue(null == dataBuffer.getVariablesThatContain("1234567890", true, currentlyMatched));
      assertTrue(null == dataBuffer.getVariablesThatContain("987654321", true, currentlyMatched));

   }

   @Test // timeout = 300000
   public void testGetVariablesThatStartWith()
   {

      YoDouble yoVariable1 = new YoDouble("doy", registry);
      YoDouble yoVariable2 = new YoDouble("Dog", registry);
      YoDouble yoVariable3 = new YoDouble("bar", registry);

      dataBuffer.addVariable(yoDouble);
      dataBuffer.addVariable(yoBoolean);
      dataBuffer.addVariable(yoInteger);
      dataBuffer.addVariable(yoEnum);
      dataBuffer.addVariable(yoVariable1);
      dataBuffer.addVariable(yoVariable2);
      dataBuffer.addVariable(yoVariable3);

      assertTrue(4 == dataBuffer.getVariablesThatStartWith("y").size());
      assertTrue(2 == dataBuffer.getVariablesThatStartWith("d", false).size());
      assertTrue(1 == dataBuffer.getVariablesThatStartWith("b").size());

   }

   @Test // timeout = 300000
   public void testGetEntries()
   {
      ArrayList<DataBufferEntry> expectedDataEntries = new ArrayList<>();
      DataBufferEntry doubleDataBufferEntryTest = new DataBufferEntry(yoDouble, testBufferSize);
      DataBufferEntry booleanDataBufferEntryTest = new DataBufferEntry(yoBoolean, testBufferSize);
      DataBufferEntry integerDataBufferEntryTest = new DataBufferEntry(yoInteger, testBufferSize);
      DataBufferEntry enumDataBufferEntryTest = new DataBufferEntry(yoEnum, testBufferSize);

      dataBuffer.addEntry(doubleDataBufferEntryTest);
      dataBuffer.addEntry(booleanDataBufferEntryTest);
      dataBuffer.addEntry(integerDataBufferEntryTest);
      dataBuffer.addEntry(enumDataBufferEntryTest);

      expectedDataEntries.add(doubleDataBufferEntryTest);
      expectedDataEntries.add(booleanDataBufferEntryTest);
      expectedDataEntries.add(integerDataBufferEntryTest);
      expectedDataEntries.add(enumDataBufferEntryTest);

      assertEquals(expectedDataEntries, dataBuffer.getEntries());
   }

   @Test // timeout = 300000
   public void testGetVariables()
   {
      dataBuffer.addVariable(yoDouble, testBufferSize);
      dataBuffer.addVariable(yoBoolean, testBufferSize);
      dataBuffer.addVariable(yoInteger, testBufferSize);
      dataBuffer.addVariable(yoEnum, testBufferSize);

      ArrayList<YoVariable<?>> expectedArrayOfVariables = new ArrayList<>();
      expectedArrayOfVariables.add(yoDouble);
      expectedArrayOfVariables.add(yoBoolean);
      expectedArrayOfVariables.add(yoInteger);
      expectedArrayOfVariables.add(yoEnum);

      List<YoVariable<?>> actualArrayOfVariables = dataBuffer.getVariables();

      for (int i = 0; i < actualArrayOfVariables.size(); i++)
      {
         assertTrue(expectedArrayOfVariables.contains(actualArrayOfVariables.get(i)));
      }

   }

   @Test // timeout = 300000
   public void testEmptyBufferIncreaseBufferSize()
   {
      int originalBufferSize = dataBuffer.getBufferSize();
      int newBufferSize = originalBufferSize * 2;

      dataBuffer.changeBufferSize(newBufferSize);
      //      .println(newBufferSize + " " + dataBuffer.getBufferSize());
      assertEquals(newBufferSize, dataBuffer.getBufferSize());
   }

   @Test // timeout = 300000
   public void testEmptyBufferDecreaseBufferSize()
   {
      int originalBufferSize = dataBuffer.getBufferSize();
      int newBufferSize = originalBufferSize / 2;

      dataBuffer.changeBufferSize(newBufferSize);
      //      .println(newBufferSize + " " + dataBuffer.getBufferSize());
      assertEquals(newBufferSize, dataBuffer.getBufferSize());
   }

   @Test // timeout = 300000
   public void testEnlargeBufferSize()
   {
      DataBufferEntry doubleDataBufferEntryTest = new DataBufferEntry(yoDouble, testBufferSize);
      dataBuffer.addEntry(doubleDataBufferEntryTest);

      int originalBufferSize = dataBuffer.getBufferSize();
      int newBufferSize = originalBufferSize * 2;

      dataBuffer.changeBufferSize(newBufferSize);
      //      System.out.println(newBufferSize + " " + dataBuffer.getBufferSize());
      assertEquals(newBufferSize, dataBuffer.getBufferSize());
   }

   @Test // timeout = 300000
   public void testDecreaseBufferSize()
   {
      DataBufferEntry doubleDataBufferEntryTest = new DataBufferEntry(yoDouble, testBufferSize);
      dataBuffer.addEntry(doubleDataBufferEntryTest);

      int originalBufferSize = dataBuffer.getBufferSize();
      int newBufferSize = originalBufferSize / 2;

      dataBuffer.changeBufferSize(newBufferSize);
      //      System.out.println(newBufferSize + " " + dataBuffer.getBufferSize());
      assertEquals(newBufferSize, dataBuffer.getBufferSize());
   }

   @Test // timeout = 300000
   public void testTick()
   {
      int numberOfTicksAndUpdates = 20;
      for (int i = 0; i < numberOfTicksAndUpdates; i++)
      {
         dataBuffer.tickAndUpdate();
      }

      dataBuffer.gotoInPoint();

      int expectedIndex = 0;
      while (dataBuffer.getIndex() < dataBuffer.getBufferInOutLength() - 1)
      {
         assertEquals(expectedIndex, dataBuffer.getIndex());
         boolean rolledOver = dataBuffer.tick(1);
         assertFalse(rolledOver);
         expectedIndex++;
      }

      boolean rolledOver = dataBuffer.tick(1);
      assertTrue(rolledOver);
      expectedIndex = 0;
      assertEquals(expectedIndex, dataBuffer.getIndex());

      rolledOver = dataBuffer.tick(1);
      assertFalse(rolledOver);
      expectedIndex = 1;
      assertEquals(expectedIndex, dataBuffer.getIndex());

   }

   @Test // timeout = 300000
   public void testIsIndexBetweenInAndOutPoint()
   {
      assertEquals(0, dataBuffer.getIndex());
      assertEquals(0, dataBuffer.getInPoint());
      assertEquals(0, dataBuffer.getOutPoint());
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(0));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(1));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(-1));

      dataBuffer.tickAndUpdate();
      assertEquals(1, dataBuffer.getIndex());
      assertEquals(0, dataBuffer.getInPoint());
      assertEquals(1, dataBuffer.getOutPoint());
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(0));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(1));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(-1));

      dataBuffer.tickAndUpdate();
      assertEquals(2, dataBuffer.getIndex());
      assertEquals(0, dataBuffer.getInPoint());
      assertEquals(2, dataBuffer.getOutPoint());
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(0));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(1));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(2));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(3));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(-1));

      int numTicks = 20;
      for (int i = 0; i < numTicks; i++)
      {
         dataBuffer.tickAndUpdate();
      }
      assertEquals(dataBuffer.getOutPoint(), dataBuffer.getIndex());
      assertEquals(0, dataBuffer.getInPoint());

      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex() - 1));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex()));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex() + 1));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getInPoint() - 1));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getInPoint()));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getInPoint() + 1));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getOutPoint() - 1));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getOutPoint()));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getOutPoint() + 1));

      dataBuffer.cropData();
      dataBuffer.gotoOutPoint();

      dataBuffer.setWrapBuffer(true);

      assertEquals(dataBuffer.getOutPoint(), dataBuffer.getIndex());
      assertEquals(0, dataBuffer.getInPoint());

      numTicks = 7;

      for (int i = 0; i < numTicks; i++)
      {
         dataBuffer.tickAndUpdate();
      }

      assertEquals(dataBuffer.getOutPoint(), dataBuffer.getIndex());
      assertEquals(numTicks - 1, dataBuffer.getOutPoint());
      assertEquals(numTicks, dataBuffer.getInPoint());

      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex() - 1));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex()));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex() + 1));

      dataBuffer.tickAndUpdate();
      assertEquals(dataBuffer.getOutPoint(), dataBuffer.getIndex());
      assertEquals(dataBuffer.getOutPoint(), dataBuffer.getInPoint() - 1);

      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex() - 1));
      assertTrue(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex()));
      assertFalse(dataBuffer.isIndexBetweenInAndOutPoint(dataBuffer.getIndex() + 1));

   }

   @Test // timeout = 300000
   public void testSetSafeToChangeIndex() //Luke Morris
   {
      boolean isSafe = dataBuffer.isSafeToChangeIndex();
      assertTrue(isSafe);
      dataBuffer.setSafeToChangeIndex(false);
      boolean isNowSafe = dataBuffer.isSafeToChangeIndex();
      assertFalse(isNowSafe);
      dataBuffer.setSafeToChangeIndex(true);
      boolean isFinallySafe = dataBuffer.isSafeToChangeIndex();
      assertTrue(isFinallySafe);
   }

   @Test // timeout = 300000
   public void testGetVars() //Luke Morris

   {
      dataBuffer.addEntry(aBuffer);
      dataBuffer.addEntry(bBuffer);
      dataBuffer.addEntry(cBuffer);

      String[] varNames = new String[3];
      varNames[0] = "a_arm";
      varNames[1] = "b_arm";
      varNames[2] = "c_arm";

      String[] aNames = new String[1];
      aNames[0] = "a_arm";

      String[] allRegularExpressions = {".*"};
      String[] cRegularExpressions = {"c.*"};

      List<YoVariable<?>> both = dataBuffer.getVars(varNames, allRegularExpressions);

      assertTrue(both.contains(a));
      assertTrue(both.contains(b));
      assertTrue(both.contains(c));

      List<YoVariable<?>> justNames = dataBuffer.getVars(varNames, null);

      assertTrue(justNames.contains(a));
      assertTrue(justNames.contains(b));
      assertTrue(justNames.contains(c));

      List<YoVariable<?>> justA = dataBuffer.getVars(aNames, null);

      assertTrue(justA.contains(a));
      assertFalse(justA.contains(b));
      assertFalse(justA.contains(c));

      List<YoVariable<?>> justRegExp = dataBuffer.getVars(null, allRegularExpressions);

      assertTrue(justRegExp.contains(a));
      assertTrue(justRegExp.contains(b));
      assertTrue(justRegExp.contains(c));

      List<YoVariable<?>> cRegExp = dataBuffer.getVars(null, cRegularExpressions);

      assertFalse(cRegExp.contains(a));
      assertFalse(cRegExp.contains(b));
      assertTrue(cRegExp.contains(c));

      List<YoVariable<?>> neither = dataBuffer.getVars(null, null);

      assertFalse(neither.contains(a));
      assertFalse(neither.contains(b));
      assertFalse(neither.contains(c));
   }

   @Test // timeout = 300000
   public void testSetMaxBufferSize()

   {
      int minBuffer = 1;
      int maxBuffer = 10000000;
      int zeroBuffer = 0;
      int normalBuffer = 200;

      dataBuffer.setMaxBufferSize(minBuffer);
      assertTrue(dataBuffer.getMaxBufferSize() == minBuffer);

      dataBuffer.setMaxBufferSize(maxBuffer);
      assertTrue(dataBuffer.getMaxBufferSize() == maxBuffer);

      dataBuffer.setMaxBufferSize(zeroBuffer);
      assertTrue(dataBuffer.getMaxBufferSize() == zeroBuffer);

      dataBuffer.setMaxBufferSize(normalBuffer);
      assertTrue(dataBuffer.getMaxBufferSize() == normalBuffer);

   }

   @Test // timeout = 300000
   public void testResetDataBuffer()
   {
      dataBuffer.addEntry(aBuffer);
      dataBuffer.addEntry(bBuffer);
      dataBuffer.addEntry(cBuffer);

      List<YoVariable<?>> withVariables = dataBuffer.getVariables();

      //      System.out.println(withVariables.size());
      assertTrue(withVariables.size() > 0);

      dataBuffer.resetDataBuffer();
   }

   @Test // timeout = 30000
   public void testCloseAndDispose()
   {
      dataBuffer.addEntry(aBuffer);
      dataBuffer.addEntry(bBuffer);
      dataBuffer.addEntry(cBuffer);
      assertTrue(dataBuffer.getEntries().size() == 3);
      dataBuffer.closeAndDispose();
      assertTrue(dataBuffer.getEntries() == null);
      assertTrue(dataBuffer.getIndex() == -1);
   }

   @Test // timeout = 30000
   public void testCopyValuesThrough()
   {
      Random random = new Random(574893);
      fillDataBufferWithRandomData(random);

      List<DataBufferEntry> entries = dataBuffer.getEntries();

      // check that each point for each entry is filled with random data
      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry dataBufferEntry = entries.get(i);
         double[] data = dataBufferEntry.getData();
         for (int j = 0; j < data.length - 1; j++)
         {
            // assuming that 0.0 wasn't randomly generated
            assertTrue(data[j] != 0.0);
         }
      }

      // method being tested: replace each data point in each DataBufferEntry with the current value of the
      // YoVariable assigned to that DataBufferEntry
      dataBuffer.copyValuesThrough();

      // each point for each entry should now equal the current value of the entry's YoVariable
      for (int i = 0; i < entries.size(); i++)
      {
         DataBufferEntry dataBufferEntry = entries.get(i);
         YoVariable<?> variable = dataBufferEntry.getVariable();
         double[] data = dataBufferEntry.getData();
         for (int j = 0; j < data.length - 1; j++)
         {
            assertTrue(data[j] == variable.getValueAsDouble());
         }
      }
   }

   @Test // timeout = 30000
   public void testPackDataWithInvalidStartPoint()
   {
      Random random = new Random(27093);
      fillDataBufferWithRandomData(random);

      // assert inPoint at start of buffer, outPoint at the end
      assertTrue(dataBuffer.getInPoint() == 0);
      assertTrue(dataBuffer.getOutPoint() == testBufferSize - 1);

      // assert in/outPoint don't chnage if start point is outside buffer range
      dataBuffer.packData(-1);
      assertTrue(dataBuffer.getInPoint() == 0);
      assertTrue(dataBuffer.getOutPoint() == testBufferSize - 1);

      dataBuffer.packData(testBufferSize);
      assertTrue(dataBuffer.getInPoint() == 0);
      assertTrue(dataBuffer.getOutPoint() == testBufferSize - 1);
   }

   @Test // timeout = 30000
   public void testPackData()
   {
      int TEST_ITERATIONS = 1000;
      Random random = new Random(209390);
      fillDataBufferWithRandomData(random);

      // assert inPoint at start of buffer, outPoint at the end
      assertTrue(dataBuffer.getInPoint() == 0);
      assertTrue(dataBuffer.getOutPoint() == testBufferSize - 1);

      // create a copy of the DataBuffer before packing for comparison afterwards
      DataBuffer dataBufferClone = cloneDataBuffer(dataBuffer);

      for (int i = 0; i < TEST_ITERATIONS; i++)
      {
         int newIndex = random.nextInt(testBufferSize);
         dataBuffer.setIndex(newIndex);
         int newStartLocation = random.nextInt(testBufferSize);
         dataBuffer.packData(newStartLocation);

         List<DataBufferEntry> entries = dataBuffer.getEntries();
         List<DataBufferEntry> entriesClone = dataBufferClone.getEntries();
         for (int j = 0; j < dataBuffer.getEntries().size(); j++)
         {
            DataBufferEntry entry = entries.get(j);
            DataBufferEntry entryClone = entriesClone.get(j);

            double[] data = entry.getData();
            double[] dataClone = entryClone.getData();

            for (int k = 0; k < data.length; k++)
            {
               assertTrue(dataClone[k] == data[(k + testBufferSize - newStartLocation) % testBufferSize]);
            }

            if (newStartLocation >= newIndex)
               assertTrue(dataBuffer.getIndex() == 0);
            else
               assertTrue(dataBuffer.getIndex() == newIndex - newStartLocation);
            assertTrue(dataBuffer.getInPoint() == 0);
            assertTrue(dataBuffer.getOutPoint() == testBufferSize - 1 - newStartLocation);

            entryClone.setData(entry.getData(), entry.getDataLength());
         }

         dataBuffer.setInOutPointFullBuffer();
      }
   }

   private void fillDataBufferWithRandomData(Random random)
   {
      // add entries to the buffer and set the in-out points so the whole buffer is used
      dataBuffer.addEntry(aBuffer);
      dataBuffer.addEntry(bBuffer);
      dataBuffer.addEntry(cBuffer);
      dataBuffer.setInOutPointFullBuffer();

      // fill entries with random data
      for (int i = 0; i < testBufferSize; i++)
      {
         a.set(random.nextDouble());
         b.set(random.nextDouble());
         c.set(random.nextDouble());
         dataBuffer.updateAndTick();
      }
   }

   @Test // timeout = 30000
   public void testCheckIfDataIsEqual()
   {
      DataBuffer dataBuffer = new DataBuffer(0);
      DataBuffer otherDataBuffer = new DataBuffer(0);

      assertTrue(dataBuffer.checkIfDataIsEqual(otherDataBuffer, 1e-6));

      YoVariableRegistry dataBufferRegistry = new YoVariableRegistry("dataBufferRegistry");
      YoDouble dataBufferYoDouble = new YoDouble("dataBufferYoDouble", dataBufferRegistry);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(dataBufferYoDouble, 0);
      dataBuffer.addEntry(dataBufferEntry);

      assertFalse(dataBuffer.checkIfDataIsEqual(otherDataBuffer, 1));

      YoVariableRegistry otherDataBufferRegistry = new YoVariableRegistry("otherDataBufferRegistry");
      YoDouble otherDataBufferYoDouble = new YoDouble("otherDataBufferYoDouble", otherDataBufferRegistry);
      DataBufferEntry otherDataBufferEntry = new DataBufferEntry(otherDataBufferYoDouble, 0);
      otherDataBuffer.addEntry(otherDataBufferEntry);

      assertFalse(dataBuffer.checkIfDataIsEqual(otherDataBuffer, 1e-6));

      dataBuffer = new DataBuffer(testBufferSize);
      otherDataBuffer = new DataBuffer(testBufferSize);

      dataBuffer.addVariable(dataBufferYoDouble);
      otherDataBuffer.addVariable(dataBufferYoDouble);

      int numberOfTicks = 5;
      for (int i = 0; i < numberOfTicks; i++)
      {
         dataBufferYoDouble.set(1.0);
         dataBuffer.tickAndUpdate();

         dataBufferYoDouble.set(0.0);
         otherDataBuffer.tickAndUpdate();
      }

      assertFalse(dataBuffer.checkIfDataIsEqual(otherDataBuffer, 1e-6));
   }

   @Test // timeout = 30000
   public void testCloneDataBuffer()
   {
      Random random = new Random(19824);
      fillDataBufferWithRandomData(random);
      DataBuffer dataBufferClone = cloneDataBuffer(dataBuffer);

      assertTrue(dataBuffer.checkIfDataIsEqual(dataBufferClone, 1e-6));
   }

   private DataBuffer cloneDataBuffer(DataBuffer originalDataBuffer)
   {
      DataBuffer cloneDataBuffer = new DataBuffer(originalDataBuffer.getBufferSize());

      for (DataBufferEntry originalEntry : originalDataBuffer.getEntries())
      {
         DataBufferEntry cloneEntry = new DataBufferEntry(originalEntry.getVariable(), originalEntry.getDataLength());
         cloneEntry.setData(originalEntry.getData(), originalEntry.getDataLength());
         cloneDataBuffer.addEntry(cloneEntry);
      }

      cloneDataBuffer.setIndex(originalDataBuffer.getIndex());
      cloneDataBuffer.setInPoint(originalDataBuffer.getInPoint());
      cloneDataBuffer.setOutPoint(originalDataBuffer.getOutPoint());

      return cloneDataBuffer;
   }

   @Test // timeout = 30000
   public void testCutDataWithInvalidStartAndEnd()
   {
      Random random = new Random(6543897);
      fillDataBufferWithRandomData(random);

      assertTrue(dataBuffer.getIndex() == 0);
      assertTrue(dataBuffer.getInPoint() == 0);
      assertTrue(dataBuffer.getOutPoint() == testBufferSize - 1);

      // attempt to cut data with unreasonable start point
      dataBuffer.cutData(-1, testBufferSize / 2);

      // assert nothing changed
      assertTrue(dataBuffer.getIndex() == 0);
      assertTrue(dataBuffer.getInPoint() == 0);
      assertTrue(dataBuffer.getOutPoint() == testBufferSize - 1);

      // attempt to cut data with unreasonable end point
      dataBuffer.cutData(testBufferSize / 2, testBufferSize + 1);

      // assert nothing changed
      assertTrue(dataBuffer.getIndex() == 0);
      assertTrue(dataBuffer.getInPoint() == 0);
      assertTrue(dataBuffer.getOutPoint() == testBufferSize - 1);
   }

   @Test // timeout = 30000
   public void testCutDataOfEntireBuffer()
   {
      Random random = new Random(27489);
      fillDataBufferWithRandomData(random);

      // assert that the in and out points are at the edges of the buffer
      assertTrue(dataBuffer.getInPoint() == 0);
      assertTrue(dataBuffer.getOutPoint() == testBufferSize - 1);

      // the no-arg constructor should use the in/out points for the start and end of the region to cut
      // this should cut the entire buffer and effectively erase all of its data
      dataBuffer.cutData();

      // assert that the length of the buffer didn't change
      // this is true when the entire buffer is cut
      assertTrue(dataBuffer.getBufferSize() == testBufferSize);

      for (DataBufferEntry entry : dataBuffer.getEntries())
      {
         for (double d : entry.getData())
         {
            assertTrue(d == 0);
         }
      }
   }

   @Test // timeout = 30000
   public void testCutData()
   {
      int TEST_ITERATIONS = 1000;
      Random random = new Random(345890);
      fillDataBufferWithRandomData(random);

      // assert that the in and out points are at the edges of the buffer
      assertTrue(dataBuffer.getInPoint() == 0);
      assertTrue(dataBuffer.getOutPoint() == testBufferSize - 1);

      // Create a copy of dataBuffer before it is modified
      DataBuffer unmodifiedDataBuffer = cloneDataBuffer(dataBuffer);

      for (int i = 0; i < TEST_ITERATIONS; i++)
      {
         int start = random.nextInt(testBufferSize);
         int end = random.nextInt(testBufferSize);

         dataBuffer.cutData(start, end);

         for (int j = 0; j < dataBuffer.getEntries().size(); j++)
         {
            DataBufferEntry cutEntry = dataBuffer.getEntries().get(j);
            DataBufferEntry unmodifiedEntry = unmodifiedDataBuffer.getEntries().get(j);

            for (int k = 0; k < cutEntry.getData().length; k++)
            {
               if (start < end)
               {
                  // First check that the size of the buffer has been cut to the expected size
                  assertTrue(dataBuffer.getBufferSize() == testBufferSize - (end - start + 1));
                  double dataFromCutEntry = cutEntry.getData()[k];

                  int indexInUnmodifiedEntry = k < start ? k : end + k - start + 1;
                  double dataFromUnmodifiedEntry = unmodifiedEntry.getData()[indexInUnmodifiedEntry];

                  // Check that only data outside of the cut-range remains in the buffer
                  assertTrue(dataFromCutEntry == dataFromUnmodifiedEntry);
               }
               else
               {
                  if (start > end)
                  {
                     // This is considered an invalid cut. DataBuffer should be unchanged
                     assertTrue(dataBuffer.getBufferSize() == testBufferSize);
                     assertTrue(cutEntry.getData()[k] == unmodifiedEntry.getData()[k]);
                  }
                  else
                  {
                     // Here start = end. Only cut the single data-point
                     assertTrue(dataBuffer.getBufferSize() == testBufferSize - 1);
                     double dataFromCutEntry = cutEntry.getData()[k];

                     int indexInUnmodifiedEntry = k < start ? k : k + 1;
                     double dataFromUnmodifiedEntry = unmodifiedEntry.getData()[indexInUnmodifiedEntry];

                     // Check that only data outside of the cut-range remains in the buffer
                     assertTrue(dataFromCutEntry == dataFromUnmodifiedEntry);
                  }
               }
            }
         }

         // Restore dataBuffer for the next iteration
         dataBuffer = cloneDataBuffer(unmodifiedDataBuffer);
      }
   }

   @Test // timeout = 30000
   public void testThinData()
   {
      int TEST_ITERATIONS = 1000;
      Random random = new Random(246370);
      fillDataBufferWithRandomData(random);

      DataBuffer unmodifiedDataBuffer = cloneDataBuffer(dataBuffer);

      for (int i = 0; i < TEST_ITERATIONS; i++)
      {
         int keepEveryNthPoint = random.nextInt(testBufferSize - 1) + 1;
         dataBuffer.thinData(keepEveryNthPoint);

         for (int j = 0; j < dataBuffer.getEntries().size(); j++)
         {
            DataBufferEntry thinnedEntry = dataBuffer.getEntries().get(j);
            DataBufferEntry unmodifiedEntry = unmodifiedDataBuffer.getEntries().get(j);

            for (int k = 0; k < thinnedEntry.getData().length; k++)
            {
               double[] thinnedEntryData = thinnedEntry.getData();
               double[] unmodifiedEntryData = unmodifiedEntry.getData();

               if (keepEveryNthPoint < testBufferSize / 2)
               {
                  assertTrue(thinnedEntry.getDataLength() == testBufferSize / keepEveryNthPoint);
                  double thinnedEntryDatum = thinnedEntryData[k];
                  double unmodifiedEntryDatum = unmodifiedEntryData[k * keepEveryNthPoint];
                  assertTrue(thinnedEntryDatum == unmodifiedEntryDatum);
               }
               else
               {
                  assertTrue(thinnedEntry.getDataLength() == testBufferSize);
                  assertTrue(thinnedEntryData[k] == unmodifiedEntryData[k]);
               }
            }
         }

         dataBuffer = cloneDataBuffer(unmodifiedDataBuffer);
      }
   }

   @Test // timeout = 30000
   public void testAttachSimulationRewoundListeners()
   {
      final boolean[] listenerNotified = {false, false, false};

      ArrayList<RewoundListener> simulationRewoundListeners = new ArrayList<>();
      simulationRewoundListeners.add(() -> listenerNotified[0] = true);
      simulationRewoundListeners.add(() -> listenerNotified[1] = true);

      dataBuffer.attachSimulationRewoundListeners(simulationRewoundListeners);

      assertFalse(listenerNotified[0]);
      assertFalse(listenerNotified[1]);
      assertFalse(listenerNotified[2]);

      dataBuffer.tickButDoNotNotifySimulationRewoundListeners(1);

      assertFalse(listenerNotified[0]);
      assertFalse(listenerNotified[1]);
      assertFalse(listenerNotified[2]);

      dataBuffer.notifyRewindListeners();

      assertTrue(listenerNotified[0]);
      assertTrue(listenerNotified[1]);
      assertFalse(listenerNotified[2]);
   }

   @Test // timeout = 30000
   public void testAttachIndexChangedListener()
   {
      final boolean[] listenerNotified = {false, false};

      IndexChangedListener indexChangedListener = (int newIndex) -> listenerNotified[0] = true;

      dataBuffer.attachIndexChangedListener(indexChangedListener);

      assertFalse(listenerNotified[0]);
      assertFalse(listenerNotified[1]);

      dataBuffer.tickAndUpdate();

      assertTrue(listenerNotified[0]);
      assertFalse(listenerNotified[1]);
   }

   @Test // timeout = 30000
   public void testNotifyDataBufferListeners()
   {
      YoVariableRegistry registryOfInterest = new YoVariableRegistry("registryOfInterest");
      NameSpace registryOfInterestNameSpace = registryOfInterest.getNameSpace();

      int NUMBER_OF_VARIABLES_TO_ADD = 30;
      double VALUE_TO_SET = 1.0;

      for (int i = 0; i < NUMBER_OF_VARIABLES_TO_ADD; i++)
      {
         YoDouble yoDouble = new YoDouble("yoDouble_" + i, registryOfInterest);
         yoDouble.set(VALUE_TO_SET);
         dataBuffer.addVariable(yoDouble);
      }

      DataBufferListener dataBufferListener = new DataBufferListener()
      {
         @Override
         public YoDouble[] getVariablesOfInterest(YoVariableHolder yoVariableHolder)
         {
            List<YoVariable<?>> variables = yoVariableHolder.getVariables(registryOfInterestNameSpace);
            YoDouble[] ret = new YoDouble[variables.size()];
            for (int i = 0; i < ret.length; i++)
            {
               ret[i] = (YoDouble) variables.get(i);
            }

            return ret;
         }

         @Override
         public void dataBufferUpdate(double[] values)
         {
            assertTrue(values.length == NUMBER_OF_VARIABLES_TO_ADD);
            for (int i = 0; i < values.length; i++)
            {
               assertTrue(values[i] == VALUE_TO_SET);
            }
         }
      };

      dataBuffer.addDataBufferListener(dataBufferListener);
      dataBuffer.notifyRewindListeners();
   }

   @Test // timeout = 30000
   public void testApplyDataProcessingFunction()
   {
      Random random = new Random(74523);
      fillDataBufferWithRandomData(random);

      DataProcessingFunction forwardDataProcessingFunction = new DataProcessingFunction()
      {
         @Override
         public void initializeProcessing()
         {
         }

         @Override
         public void processData()
         {
            a.set(1.0);
            b.set(2.348);
            c.set(8.7834);
         }
      };

      assertFalse(a.getDoubleValue() == 1.0);
      assertFalse(b.getDoubleValue() == 2.348);
      assertFalse(c.getDoubleValue() == 8.7834);

      dataBuffer.applyDataProcessingFunction(forwardDataProcessingFunction);

      assertTrue(a.getDoubleValue() == 1.0);
      assertTrue(b.getDoubleValue() == 2.348);
      assertTrue(c.getDoubleValue() == 8.7834);

      dataBuffer.setIndex(dataBuffer.getOutPoint());

      DataProcessingFunction backwardsDataProcessingFunction = new DataProcessingFunction()
      {
         @Override
         public void initializeProcessing()
         {
         }

         @Override
         public void processData()
         {
            a.set(0.0);
            b.set(0.0);
            c.set(0.0);
         }
      };

      assertFalse(a.getDoubleValue() == 0.0);
      assertFalse(b.getDoubleValue() == 0.0);
      assertFalse(c.getDoubleValue() == 0.0);

      dataBuffer.applyDataProcessingFunctionBackward(backwardsDataProcessingFunction);

      assertTrue(a.getDoubleValue() == 0.0);
      assertTrue(b.getDoubleValue() == 0.0);
      assertTrue(c.getDoubleValue() == 0.0);
   }

   @Test // timeout = 30000
   public void testToggleKeyPointMode()
   {
      boolean keyPointModeToggled = dataBuffer.isKeyPointModeToggled();

      dataBuffer.toggleKeyPointMode();

      assertFalse(keyPointModeToggled == dataBuffer.isKeyPointModeToggled());

      dataBuffer.toggleKeyPointMode();

      assertTrue(keyPointModeToggled == dataBuffer.isKeyPointModeToggled());
   }

   @Test // timeout = 30000
   public void testGetSetTimeVariable()
   {
      String timeVariableName = "time";
      YoDouble time = new YoDouble(timeVariableName, registry);

      dataBuffer.addVariable(time);
      dataBuffer.setTimeVariableName(timeVariableName);

      int numberOfTicks = 5;

      double[] timeData = new double[numberOfTicks];

      for (int i = 0; i < numberOfTicks; i++)
      {
         time.set(i);
         timeData[i] = i;
         dataBuffer.tickAndUpdate();
      }

      assertTrue(dataBuffer.getTimeVariableName().equals(timeVariableName));

      assertTrue(timeData.length == dataBuffer.getIndex());

      double[] dataBufferTimeData = dataBuffer.getTimeData();
      for (int i = 0; i < timeData.length; i++)
      {
         // We have to add 1 to the dataBufferTimeData index because the time variable
         // already has a value of 0.0 when we call tickAndUpdate the first time
         assertTrue(timeData[i] == dataBufferTimeData[i + 1]);
      }
   }

   //testGetVars(String [], String[])

   //testGetVarsFromGroup(String varGroupName, VarGroupList varGroupList)

   //setMaxBufferSize

   //   @Test// timeout=300000
   //   public void testResetDataBuffer() throws RepeatDataBufferEntryException
   //   {
   //      dataBuffer.addNewEntry(yoDouble, testBufferSize);
   //      dataBuffer.addNewEntry(yoBoolean, testBufferSize);
   //      dataBuffer.addNewEntry(yoInteger, testBufferSize);
   //      dataBuffer.addNewEntry(yoEnum, testBufferSize);
   //
   //      dataBuffer.resetDataBuffer();
   //
   //      int dataBufferSize = dataBuffer.getBufferSize();
   //      .println(dataBufferSize);
   //      assertTrue(0 == dataBufferSize);
   //
   //   }

   //   @Test// timeout=300000
   //   public void testClearAll() throws RepeatDataBufferEntryException
   //   {
   //
   //       dataBuffer.addNewEntry(yoDouble, testBufferSize);
   //       dataBuffer.addNewEntry(yoBoolean, testBufferSize);
   //       dataBuffer.addNewEntry(yoInteger, testBufferSize);
   //       dataBuffer.addNewEntry(yoEnum, testBufferSize);
   //
   //       dataBuffer.clearAll(testBufferSize);
   //
   //       int dataBufferSize = dataBuffer.getBufferSize();
   //       .println(dataBufferSize);
   //       assertTrue(0 == dataBufferSize);
   //
   //   }
}
