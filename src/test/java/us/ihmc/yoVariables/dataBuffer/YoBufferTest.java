package us.ihmc.yoVariables.dataBuffer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoSearchTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoBufferTest
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
   private YoRegistry registry;
   private YoBuffer dataBuffer = new YoBuffer(testBufferSize);

   private YoDouble a, b, c;
   private YoBufferVariableEntry aBuffer, bBuffer, cBuffer;

   @BeforeEach
   public void setUp()
   {
      registry = new YoRegistry("testRegistry");
      yoDouble = new YoDouble("yoDouble", registry);
      yoBoolean = new YoBoolean("yoBoolean", registry);
      yoInteger = new YoInteger("yoInteger", registry);
      yoEnum = new YoEnum<>("yoEnum", registry, EnumYoVariableTestEnums.class);

      a = new YoDouble("a_arm", registry);
      b = new YoDouble("b_arm", registry);
      c = new YoDouble("c_arm", registry);

      aBuffer = new YoBufferVariableEntry(a, testBufferSize);
      bBuffer = new YoBufferVariableEntry(b, testBufferSize);
      cBuffer = new YoBufferVariableEntry(c, testBufferSize);
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
   public void testAddAndGetEntry()
   {
      YoBufferVariableEntry doubleDataBufferEntryTest = new YoBufferVariableEntry(yoDouble, testBufferSize);
      YoBufferVariableEntry booleanDataBufferEntryTest = new YoBufferVariableEntry(yoBoolean, testBufferSize);
      YoBufferVariableEntry integerDataBufferEntryTest = new YoBufferVariableEntry(yoInteger, testBufferSize);
      YoBufferVariableEntry enumDataBufferEntryTest = new YoBufferVariableEntry(yoEnum, testBufferSize);
      dataBuffer.addEntry(doubleDataBufferEntryTest);
      dataBuffer.addEntry(booleanDataBufferEntryTest);
      dataBuffer.addEntry(integerDataBufferEntryTest);
      dataBuffer.addEntry(enumDataBufferEntryTest);

      YoBufferVariableEntry testEntryReceivedViaString = dataBuffer.findVariableEntry("yoDouble");
      YoBufferVariableEntry testEntryReceivedViaVariableName = dataBuffer.getEntry(yoDouble);
      assertEquals(doubleDataBufferEntryTest, testEntryReceivedViaString);
      assertEquals(doubleDataBufferEntryTest, testEntryReceivedViaVariableName);

      testEntryReceivedViaString = dataBuffer.findVariableEntry("yoBoolean");
      testEntryReceivedViaVariableName = dataBuffer.getEntry(yoBoolean);
      assertEquals(booleanDataBufferEntryTest, testEntryReceivedViaString);
      assertEquals(booleanDataBufferEntryTest, testEntryReceivedViaVariableName);

      testEntryReceivedViaString = dataBuffer.findVariableEntry("yoInteger");
      testEntryReceivedViaVariableName = dataBuffer.getEntry(yoInteger);
      assertEquals(integerDataBufferEntryTest, testEntryReceivedViaString);
      assertEquals(integerDataBufferEntryTest, testEntryReceivedViaVariableName);

      testEntryReceivedViaString = dataBuffer.findVariableEntry("yoEnum");
      testEntryReceivedViaVariableName = dataBuffer.getEntry(yoEnum);
      assertEquals(enumDataBufferEntryTest, testEntryReceivedViaString);
      assertEquals(enumDataBufferEntryTest, testEntryReceivedViaVariableName);

   }

   @Test // timeout = 300000
   public void testAddNewEntry()
   {
      dataBuffer.addVariable(yoDouble);
      dataBuffer.addVariable(yoBoolean);
      dataBuffer.addVariable(yoInteger);
      dataBuffer.addVariable(yoEnum);

      YoBufferVariableEntry doubleDataBufferEntryTest = new YoBufferVariableEntry(yoDouble, testBufferSize);
      YoBufferVariableEntry booleanDataBufferEntryTest = new YoBufferVariableEntry(yoBoolean, testBufferSize);
      YoBufferVariableEntry integerDataBufferEntryTest = new YoBufferVariableEntry(yoInteger, testBufferSize);
      YoBufferVariableEntry enumDataBufferEntryTest = new YoBufferVariableEntry(yoEnum, testBufferSize);

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
      ArrayList<YoVariable> arrayListToBeAdded = new ArrayList<>();
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
   }

   @Test // timeout = 300000
   public void testGetEntries()
   {
      ArrayList<YoBufferVariableEntry> expectedDataEntries = new ArrayList<>();
      YoBufferVariableEntry doubleDataBufferEntryTest = new YoBufferVariableEntry(yoDouble, testBufferSize);
      YoBufferVariableEntry booleanDataBufferEntryTest = new YoBufferVariableEntry(yoBoolean, testBufferSize);
      YoBufferVariableEntry integerDataBufferEntryTest = new YoBufferVariableEntry(yoInteger, testBufferSize);
      YoBufferVariableEntry enumDataBufferEntryTest = new YoBufferVariableEntry(yoEnum, testBufferSize);

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
      dataBuffer.addVariable(yoDouble);
      dataBuffer.addVariable(yoBoolean);
      dataBuffer.addVariable(yoInteger);
      dataBuffer.addVariable(yoEnum);

      ArrayList<YoVariable> expectedArrayOfVariables = new ArrayList<>();
      expectedArrayOfVariables.add(yoDouble);
      expectedArrayOfVariables.add(yoBoolean);
      expectedArrayOfVariables.add(yoInteger);
      expectedArrayOfVariables.add(yoEnum);

      List<YoVariable> actualArrayOfVariables = dataBuffer.getVariables();

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

      dataBuffer.resizeBuffer(newBufferSize);
      //      .println(newBufferSize + " " + dataBuffer.getBufferSize());
      assertEquals(newBufferSize, dataBuffer.getBufferSize());
   }

   @Test // timeout = 300000
   public void testEmptyBufferDecreaseBufferSize()
   {
      int originalBufferSize = dataBuffer.getBufferSize();
      int newBufferSize = originalBufferSize / 2;

      dataBuffer.resizeBuffer(newBufferSize);
      //      .println(newBufferSize + " " + dataBuffer.getBufferSize());
      assertEquals(newBufferSize, dataBuffer.getBufferSize());
   }

   @Test // timeout = 300000
   public void testEnlargeBufferSize()
   {
      YoBufferVariableEntry doubleDataBufferEntryTest = new YoBufferVariableEntry(yoDouble, testBufferSize);
      dataBuffer.addEntry(doubleDataBufferEntryTest);

      int originalBufferSize = dataBuffer.getBufferSize();
      int newBufferSize = originalBufferSize * 2;

      dataBuffer.resizeBuffer(newBufferSize);
      //      System.out.println(newBufferSize + " " + dataBuffer.getBufferSize());
      assertEquals(newBufferSize, dataBuffer.getBufferSize());
   }

   @Test // timeout = 300000
   public void testDecreaseBufferSize()
   {
      YoBufferVariableEntry doubleDataBufferEntryTest = new YoBufferVariableEntry(yoDouble, testBufferSize);
      dataBuffer.addEntry(doubleDataBufferEntryTest);

      int originalBufferSize = dataBuffer.getBufferSize();
      int newBufferSize = originalBufferSize / 2;

      dataBuffer.resizeBuffer(newBufferSize);
      //      System.out.println(newBufferSize + " " + dataBuffer.getBufferSize());
      assertEquals(newBufferSize, dataBuffer.getBufferSize());
   }

   @Test // timeout = 300000
   public void testTick()
   {
      int numberOfTicksAndUpdates = 20;
      for (int i = 0; i < numberOfTicksAndUpdates; i++)
      {
         dataBuffer.tickAndWriteIntoBuffer();
      }

      dataBuffer.gotoInPoint();

      int expectedIndex = 0;
      while (dataBuffer.getCurrentIndex() < dataBuffer.getBufferInOutLength() - 1)
      {
         assertEquals(expectedIndex, dataBuffer.getCurrentIndex());
         boolean rolledOver = dataBuffer.tickAndReadFromBuffer(1);
         assertFalse(rolledOver);
         expectedIndex++;
      }

      boolean rolledOver = dataBuffer.tickAndReadFromBuffer(1);
      assertTrue(rolledOver);
      expectedIndex = 0;
      assertEquals(expectedIndex, dataBuffer.getCurrentIndex());

      rolledOver = dataBuffer.tickAndReadFromBuffer(1);
      assertFalse(rolledOver);
      expectedIndex = 1;
      assertEquals(expectedIndex, dataBuffer.getCurrentIndex());

   }

   @Test // timeout = 300000
   public void testIsIndexBetweenInAndOutPoint()
   {
      assertEquals(0, dataBuffer.getCurrentIndex());
      assertEquals(0, dataBuffer.getInPoint());
      assertEquals(0, dataBuffer.getOutPoint());
      assertTrue(dataBuffer.isIndexBetweenBounds(0));
      assertFalse(dataBuffer.isIndexBetweenBounds(1));
      assertFalse(dataBuffer.isIndexBetweenBounds(-1));

      dataBuffer.tickAndWriteIntoBuffer();
      assertEquals(1, dataBuffer.getCurrentIndex());
      assertEquals(0, dataBuffer.getInPoint());
      assertEquals(1, dataBuffer.getOutPoint());
      assertTrue(dataBuffer.isIndexBetweenBounds(0));
      assertTrue(dataBuffer.isIndexBetweenBounds(1));
      assertFalse(dataBuffer.isIndexBetweenBounds(-1));

      dataBuffer.tickAndWriteIntoBuffer();
      assertEquals(2, dataBuffer.getCurrentIndex());
      assertEquals(0, dataBuffer.getInPoint());
      assertEquals(2, dataBuffer.getOutPoint());
      assertTrue(dataBuffer.isIndexBetweenBounds(0));
      assertTrue(dataBuffer.isIndexBetweenBounds(1));
      assertTrue(dataBuffer.isIndexBetweenBounds(2));
      assertFalse(dataBuffer.isIndexBetweenBounds(3));
      assertFalse(dataBuffer.isIndexBetweenBounds(-1));

      int numTicks = 20;
      for (int i = 0; i < numTicks; i++)
      {
         dataBuffer.tickAndWriteIntoBuffer();
      }
      assertEquals(dataBuffer.getOutPoint(), dataBuffer.getCurrentIndex());
      assertEquals(0, dataBuffer.getInPoint());

      assertTrue(dataBuffer.isIndexBetweenBounds(dataBuffer.getCurrentIndex() - 1));
      assertTrue(dataBuffer.isIndexBetweenBounds(dataBuffer.getCurrentIndex()));
      assertFalse(dataBuffer.isIndexBetweenBounds(dataBuffer.getCurrentIndex() + 1));
      assertFalse(dataBuffer.isIndexBetweenBounds(dataBuffer.getInPoint() - 1));
      assertTrue(dataBuffer.isIndexBetweenBounds(dataBuffer.getInPoint()));
      assertTrue(dataBuffer.isIndexBetweenBounds(dataBuffer.getInPoint() + 1));
      assertTrue(dataBuffer.isIndexBetweenBounds(dataBuffer.getOutPoint() - 1));
      assertTrue(dataBuffer.isIndexBetweenBounds(dataBuffer.getOutPoint()));
      assertFalse(dataBuffer.isIndexBetweenBounds(dataBuffer.getOutPoint() + 1));

      dataBuffer.cropData();
      dataBuffer.gotoOutPoint();

      assertEquals(dataBuffer.getOutPoint(), dataBuffer.getCurrentIndex());
      assertEquals(0, dataBuffer.getInPoint());

      numTicks = 7;

      for (int i = 0; i < numTicks; i++)
      {
         dataBuffer.tickAndWriteIntoBuffer();
      }

      assertEquals(dataBuffer.getOutPoint(), dataBuffer.getCurrentIndex());
      assertEquals(numTicks - 1, dataBuffer.getOutPoint());
      assertEquals(numTicks, dataBuffer.getInPoint());

      assertTrue(dataBuffer.isIndexBetweenBounds(dataBuffer.getCurrentIndex() - 1));
      assertTrue(dataBuffer.isIndexBetweenBounds(dataBuffer.getCurrentIndex()));
      assertFalse(dataBuffer.isIndexBetweenBounds(dataBuffer.getCurrentIndex() + 1));

      dataBuffer.tickAndWriteIntoBuffer();
      assertEquals(dataBuffer.getOutPoint(), dataBuffer.getCurrentIndex());
      assertEquals(dataBuffer.getOutPoint(), dataBuffer.getInPoint() - 1);

      assertTrue(dataBuffer.isIndexBetweenBounds(dataBuffer.getCurrentIndex() - 1));
      assertTrue(dataBuffer.isIndexBetweenBounds(dataBuffer.getCurrentIndex()));
      assertFalse(dataBuffer.isIndexBetweenBounds(dataBuffer.getCurrentIndex() + 1));

   }

   @Test // timeout = 300000
   public void testSetSafeToChangeIndex() //Luke Morris
   {
      boolean isSafe = dataBuffer.isIndexLocked();
      assertTrue(isSafe);
      dataBuffer.setLockIndex(false);
      boolean isNowSafe = dataBuffer.isIndexLocked();
      assertFalse(isNowSafe);
      dataBuffer.setLockIndex(true);
      boolean isFinallySafe = dataBuffer.isIndexLocked();
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

      List<YoVariable> justNames = Stream.of(varNames).flatMap(varName -> dataBuffer.findVariables(varName).stream()).collect(Collectors.toList());

      assertTrue(justNames.contains(a));
      assertTrue(justNames.contains(b));
      assertTrue(justNames.contains(c));

      List<YoVariable> justA = Stream.of(aNames).flatMap(varName -> dataBuffer.findVariables(varName).stream()).collect(Collectors.toList());

      assertTrue(justA.contains(a));
      assertFalse(justA.contains(b));
      assertFalse(justA.contains(c));

      List<YoVariable> justRegExp = dataBuffer.filterVariables(YoSearchTools.regularExpressionFilter(allRegularExpressions));

      assertTrue(justRegExp.contains(a));
      assertTrue(justRegExp.contains(b));
      assertTrue(justRegExp.contains(c));

      List<YoVariable> cRegExp = dataBuffer.filterVariables(YoSearchTools.regularExpressionFilter(cRegularExpressions));

      assertFalse(cRegExp.contains(a));
      assertFalse(cRegExp.contains(b));
      assertTrue(cRegExp.contains(c));
   }

   @Test // timeout = 30000
   public void testCloseAndDispose()
   {
      dataBuffer.addEntry(aBuffer);
      dataBuffer.addEntry(bBuffer);
      dataBuffer.addEntry(cBuffer);
      assertTrue(dataBuffer.getEntries().size() == 3);
      dataBuffer.clear();
      assertTrue(dataBuffer.getEntries() == null);
      assertTrue(dataBuffer.getCurrentIndex() == -1);
   }

   @Test // timeout = 30000
   public void testCopyValuesThrough()
   {
      Random random = new Random(574893);
      fillDataBufferWithRandomData(random);

      List<YoBufferVariableEntry> entries = dataBuffer.getEntries();

      // check that each point for each entry is filled with random data
      for (int i = 0; i < entries.size(); i++)
      {
         YoBufferVariableEntry dataBufferEntry = entries.get(i);
         double[] data = dataBufferEntry.getBuffer();
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
         YoBufferVariableEntry dataBufferEntry = entries.get(i);
         YoVariable variable = dataBufferEntry.getVariable();
         double[] data = dataBufferEntry.getBuffer();
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

      for (int i = 0; i < TEST_ITERATIONS; i++)
      {
         YoBuffer dataBufferClone = new YoBuffer(dataBuffer);
         int newIndex = random.nextInt(testBufferSize);
         dataBuffer.setCurrentIndex(newIndex);
         int newStartLocation = random.nextInt(testBufferSize);
         dataBuffer.packData(newStartLocation);

         List<YoBufferVariableEntry> entries = dataBuffer.getEntries();
         List<YoBufferVariableEntry> entriesClone = dataBufferClone.getEntries();
         for (int j = 0; j < dataBuffer.getEntries().size(); j++)
         {
            YoBufferVariableEntry entry = entries.get(j);
            YoBufferVariableEntry entryClone = entriesClone.get(j);

            double[] data = entry.getBuffer();
            double[] dataClone = entryClone.getBuffer();

            for (int k = 0; k < data.length; k++)
            {
               assertTrue(dataClone[k] == data[(k + testBufferSize - newStartLocation) % testBufferSize]);
            }

            if (newStartLocation >= newIndex)
               assertTrue(dataBuffer.getCurrentIndex() == 0);
            else
               assertTrue(dataBuffer.getCurrentIndex() == newIndex - newStartLocation);
            assertTrue(dataBuffer.getInPoint() == 0);
            assertTrue(dataBuffer.getOutPoint() == testBufferSize - 1 - newStartLocation);
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
         dataBuffer.writeIntoBuffer();
         dataBuffer.tickAndReadFromBuffer(1);
      }
   }

   @Test // timeout = 30000
   public void testCheckIfDataIsEqual()
   {
      YoBuffer dataBuffer = new YoBuffer(0);
      YoBuffer otherDataBuffer = new YoBuffer(0);

      assertTrue(dataBuffer.checkIfDataIsEqual(otherDataBuffer, 1e-6));

      YoRegistry dataBufferRegistry = new YoRegistry("dataBufferRegistry");
      YoDouble dataBufferYoDouble = new YoDouble("dataBufferYoDouble", dataBufferRegistry);
      YoBufferVariableEntry dataBufferEntry = new YoBufferVariableEntry(dataBufferYoDouble, 0);
      dataBuffer.addEntry(dataBufferEntry);

      assertFalse(dataBuffer.checkIfDataIsEqual(otherDataBuffer, 1));

      YoRegistry otherDataBufferRegistry = new YoRegistry("otherDataBufferRegistry");
      YoDouble otherDataBufferYoDouble = new YoDouble("otherDataBufferYoDouble", otherDataBufferRegistry);
      YoBufferVariableEntry otherDataBufferEntry = new YoBufferVariableEntry(otherDataBufferYoDouble, 0);
      otherDataBuffer.addEntry(otherDataBufferEntry);

      assertFalse(dataBuffer.checkIfDataIsEqual(otherDataBuffer, 1e-6));

      dataBuffer = new YoBuffer(testBufferSize);
      otherDataBuffer = new YoBuffer(testBufferSize);

      dataBuffer.addVariable(dataBufferYoDouble);
      otherDataBuffer.addVariable(dataBufferYoDouble);

      int numberOfTicks = 5;
      for (int i = 0; i < numberOfTicks; i++)
      {
         dataBufferYoDouble.set(1.0);
         dataBuffer.tickAndWriteIntoBuffer();

         dataBufferYoDouble.set(0.0);
         otherDataBuffer.tickAndWriteIntoBuffer();
      }

      assertFalse(dataBuffer.checkIfDataIsEqual(otherDataBuffer, 1e-6));
   }

   @Test // timeout = 30000
   public void testCloneDataBuffer()
   {
      Random random = new Random(19824);
      fillDataBufferWithRandomData(random);
      YoBuffer dataBufferClone = new YoBuffer(dataBuffer);

      assertTrue(dataBuffer.checkIfDataIsEqual(dataBufferClone, 1e-6));
   }

   @Test // timeout = 30000
   public void testCutDataWithInvalidStartAndEnd()
   {
      Random random = new Random(6543897);
      fillDataBufferWithRandomData(random);

      assertTrue(dataBuffer.getCurrentIndex() == 0);
      assertTrue(dataBuffer.getInPoint() == 0);
      assertTrue(dataBuffer.getOutPoint() == testBufferSize - 1);

      // attempt to cut data with unreasonable start point
      dataBuffer.cutData(-1, testBufferSize / 2);

      // assert nothing changed
      assertTrue(dataBuffer.getCurrentIndex() == 0);
      assertTrue(dataBuffer.getInPoint() == 0);
      assertTrue(dataBuffer.getOutPoint() == testBufferSize - 1);

      // attempt to cut data with unreasonable end point
      dataBuffer.cutData(testBufferSize / 2, testBufferSize + 1);

      // assert nothing changed
      assertTrue(dataBuffer.getCurrentIndex() == 0);
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

      for (YoBufferVariableEntry entry : dataBuffer.getEntries())
      {
         for (double d : entry.getBuffer())
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
      YoBuffer unmodifiedDataBuffer = new YoBuffer(dataBuffer);

      for (int i = 0; i < TEST_ITERATIONS; i++)
      {
         int start = random.nextInt(testBufferSize);
         int end = random.nextInt(testBufferSize);

         dataBuffer.cutData(start, end);

         for (int j = 0; j < dataBuffer.getEntries().size(); j++)
         {
            YoBufferVariableEntry cutEntry = dataBuffer.getEntries().get(j);
            YoBufferVariableEntry unmodifiedEntry = unmodifiedDataBuffer.getEntries().get(j);

            for (int k = 0; k < cutEntry.getBuffer().length; k++)
            {
               if (start < end)
               {
                  // First check that the size of the buffer has been cut to the expected size
                  assertTrue(dataBuffer.getBufferSize() == testBufferSize - (end - start + 1));
                  double dataFromCutEntry = cutEntry.getBuffer()[k];

                  int indexInUnmodifiedEntry = k < start ? k : end + k - start + 1;
                  double dataFromUnmodifiedEntry = unmodifiedEntry.getBuffer()[indexInUnmodifiedEntry];

                  // Check that only data outside of the cut-range remains in the buffer
                  assertTrue(dataFromCutEntry == dataFromUnmodifiedEntry);
               }
               else
               {
                  if (start > end)
                  {
                     // This is considered an invalid cut. DataBuffer should be unchanged
                     assertTrue(dataBuffer.getBufferSize() == testBufferSize);
                     assertTrue(cutEntry.getBuffer()[k] == unmodifiedEntry.getBuffer()[k]);
                  }
                  else
                  {
                     // Here start = end. Only cut the single data-point
                     assertTrue(dataBuffer.getBufferSize() == testBufferSize - 1);
                     double dataFromCutEntry = cutEntry.getBuffer()[k];

                     int indexInUnmodifiedEntry = k < start ? k : k + 1;
                     double dataFromUnmodifiedEntry = unmodifiedEntry.getBuffer()[indexInUnmodifiedEntry];

                     // Check that only data outside of the cut-range remains in the buffer
                     assertTrue(dataFromCutEntry == dataFromUnmodifiedEntry);
                  }
               }
            }
         }

         // Restore dataBuffer for the next iteration
         dataBuffer = new YoBuffer(unmodifiedDataBuffer);
      }
   }

   @Test // timeout = 30000
   public void testThinData()
   {
      int TEST_ITERATIONS = 1000;
      Random random = new Random(246370);
      fillDataBufferWithRandomData(random);

      YoBuffer unmodifiedDataBuffer = new YoBuffer(dataBuffer);

      for (int i = 0; i < TEST_ITERATIONS; i++)
      {
         int keepEveryNthPoint = random.nextInt(testBufferSize - 1) + 1;
         dataBuffer.thinData(keepEveryNthPoint);

         for (int j = 0; j < dataBuffer.getEntries().size(); j++)
         {
            YoBufferVariableEntry thinnedEntry = dataBuffer.getEntries().get(j);
            YoBufferVariableEntry unmodifiedEntry = unmodifiedDataBuffer.getEntries().get(j);

            for (int k = 0; k < thinnedEntry.getBuffer().length; k++)
            {
               double[] thinnedEntryData = thinnedEntry.getBuffer();
               double[] unmodifiedEntryData = unmodifiedEntry.getBuffer();

               if (keepEveryNthPoint < testBufferSize / 2)
               {
                  assertTrue(thinnedEntry.getBufferSize() == testBufferSize / keepEveryNthPoint);
                  double thinnedEntryDatum = thinnedEntryData[k];
                  double unmodifiedEntryDatum = unmodifiedEntryData[k * keepEveryNthPoint];
                  assertTrue(thinnedEntryDatum == unmodifiedEntryDatum);
               }
               else
               {
                  assertTrue(thinnedEntry.getBufferSize() == testBufferSize);
                  assertTrue(thinnedEntryData[k] == unmodifiedEntryData[k]);
               }
            }
         }

         dataBuffer = new YoBuffer(unmodifiedDataBuffer);
      }
   }

   @Test // timeout = 30000
   public void testAttachIndexChangedListener()
   {
      final boolean[] listenerNotified = {false, false};

      YoBufferIndexChangedListener indexChangedListener = (int newIndex) -> listenerNotified[0] = true;

      dataBuffer.attachIndexChangedListener(indexChangedListener);

      assertFalse(listenerNotified[0]);
      assertFalse(listenerNotified[1]);

      dataBuffer.tickAndWriteIntoBuffer();

      assertTrue(listenerNotified[0]);
      assertFalse(listenerNotified[1]);
   }

   @Test // timeout = 30000
   public void testApplyDataProcessingFunction()
   {
      Random random = new Random(74523);
      fillDataBufferWithRandomData(random);

      YoBufferProcessor forwardDataProcessingFunction = new YoBufferProcessor()
      {
         @Override
         public void process(int startIndex, int endIndex, int currentIndex)
         {
            a.set(1.0);
            b.set(2.348);
            c.set(8.7834);
         }
      };

      assertFalse(a.getDoubleValue() == 1.0);
      assertFalse(b.getDoubleValue() == 2.348);
      assertFalse(c.getDoubleValue() == 8.7834);

      dataBuffer.applyProcessor(forwardDataProcessingFunction);

      assertTrue(a.getDoubleValue() == 1.0);
      assertTrue(b.getDoubleValue() == 2.348);
      assertTrue(c.getDoubleValue() == 8.7834);

      dataBuffer.setCurrentIndex(dataBuffer.getOutPoint());

      YoBufferProcessor backwardsDataProcessingFunction = new YoBufferProcessor()
      {
         @Override
         public boolean goForward()
         {
            return false;
         }
         @Override
         public void process(int startIndex, int endIndex, int currentIndex)
         {
            a.set(0.0);
            b.set(0.0);
            c.set(0.0);
         }
      };

      assertFalse(a.getDoubleValue() == 0.0);
      assertFalse(b.getDoubleValue() == 0.0);
      assertFalse(c.getDoubleValue() == 0.0);

      dataBuffer.applyProcessor(backwardsDataProcessingFunction);

      assertTrue(a.getDoubleValue() == 0.0);
      assertTrue(b.getDoubleValue() == 0.0);
      assertTrue(c.getDoubleValue() == 0.0);
   }

   @Test // timeout = 30000
   public void testToggleKeyPointMode()
   {
      boolean keyPointModeToggled = dataBuffer.getKeyPointsHandler().areKeyPointsEnabled();

      dataBuffer.getKeyPointsHandler().toggleKeyPoints();

      assertFalse(keyPointModeToggled == dataBuffer.getKeyPointsHandler().areKeyPointsEnabled());

      dataBuffer.getKeyPointsHandler().toggleKeyPoints();

      assertTrue(keyPointModeToggled == dataBuffer.getKeyPointsHandler().areKeyPointsEnabled());
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
         dataBuffer.tickAndWriteIntoBuffer();
      }

      assertTrue(dataBuffer.getTimeVariableName().equals(timeVariableName));

      assertTrue(timeData.length == dataBuffer.getCurrentIndex());

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
