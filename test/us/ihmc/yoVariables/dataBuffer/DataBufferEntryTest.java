package us.ihmc.yoVariables.dataBuffer;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.yoVariables.variable.YoDouble;

public class DataBufferEntryTest
{
   private YoDouble yoDouble;
   private DataBufferEntry dataBufferEntry;
   private int nPoints = 10000;

   @Before
   public void setup()
   {
      yoDouble = new YoDouble("yoDouble", null);
      yoDouble.set(0);
      dataBufferEntry = new DataBufferEntry(yoDouble, nPoints);
   }

   @After
   public void tearDown()
   {
      yoDouble = null;
      dataBufferEntry = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVal()
   {
      Random random = new Random(6432);

      double tempDouble = (double) random.nextInt(20000) / (double) random.nextInt(30);
      yoDouble.set(tempDouble);
      assertEquals(tempDouble, dataBufferEntry.getVariableValueAsADouble(), 0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testTickAndUpdate()
   {
      Random random = new Random(1345143);

      double epsilon = 0;
      double[] tempData = new double[nPoints];
      for (int i = 0; i < nPoints; i++)
      {
         tempData[i] = (double) random.nextInt(20000) / (double) random.nextInt(30);
         yoDouble.set(tempData[i]);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }

      double[] data = dataBufferEntry.getData();

      for (int i = 0; i < nPoints; i++)
      {
         assertEquals(tempData[i], data[i], epsilon);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000) public void testComputeAverage()
   {
      Random random = new Random(768439);

      double lowerBound = -100.0;
      double upperBound = 100.0;

      double total = 0.0;
      for (int i = 0; i < nPoints; i++)
      {
         double data = random.nextDouble() * (upperBound - lowerBound) + lowerBound;
         yoDouble.set(data);
         total = total + data;
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }

      double average = total / ((double) nPoints);
      double computedAverage = dataBufferEntry.computeAverage();

      assertEquals(average, computedAverage, 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testUpdateValue()
   {
      Random random = new Random(754380);

      double epsilon = 0;
      double[] tempData = new double[nPoints];
      for (int i = 0; i < nPoints; i++)
      {
         tempData[i] = (double) random.nextInt(20000) / (double) random.nextInt(30);
         yoDouble.set(tempData[i]);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }

      for (int i = 0; i < nPoints; i++)
      {
         dataBufferEntry.setYoVariableValueToDataAtIndex(i);
         assertEquals(yoDouble.getValueAsDouble(), dataBufferEntry.getData()[i], epsilon);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCheckIfDataIsEqual()
   {
      Random random = new Random(32890);

      double temp;
      double epsilon = 0;
      YoDouble yoDouble2 = new YoDouble("yoDouble2", null);
      yoDouble2.set(0);
      DataBufferEntry entry2 = new DataBufferEntry(yoDouble2, nPoints);

      for (int i = 0; i < nPoints; i++)
      {
         temp = (double) random.nextInt(20000) / (double) random.nextInt(30);

         yoDouble.set(temp);
         yoDouble2.set(temp);

         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
         entry2.setDataAtIndexToYoVariableValue(i);
      }

      assertTrue(dataBufferEntry.checkIfDataIsEqual(entry2, 0, nPoints - 1, epsilon));

      dataBufferEntry = entry2 = null;

      dataBufferEntry = new DataBufferEntry(yoDouble, nPoints);
      entry2 = new DataBufferEntry(yoDouble2, nPoints);

      for (int i = 0; i < nPoints; i++)
      {
         temp = (double) random.nextInt(20000) / (double) random.nextInt(30);

         yoDouble.set(temp);
         yoDouble2.set(-temp);

         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
         entry2.setDataAtIndexToYoVariableValue(i);
      }

      assertFalse(dataBufferEntry.checkIfDataIsEqual(entry2, 0, nPoints - 1, epsilon));
      assertFalse(dataBufferEntry.checkIfDataIsEqual(entry2, 0, nPoints, epsilon)); // outPoint out of bounds
      assertFalse(dataBufferEntry.checkIfDataIsEqual(entry2, nPoints, nPoints - 1, epsilon)); //inPoint out of bounds
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetMinAndMaxScaling()
   {
      Random random = new Random(80423);

      double minScaling = random.nextDouble();
      double maxScaling = random.nextDouble() + 2;
      dataBufferEntry.setManualScaling(minScaling, maxScaling);

      assertEquals(minScaling, dataBufferEntry.getManualMinScaling(), 0);
      assertEquals(maxScaling, dataBufferEntry.getManualMaxScaling(), 0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariable()
   {
      assertEquals(yoDouble, dataBufferEntry.getVariable());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCopyValueThrough()
   {
      Random random = new Random(2346180);

      double tempDouble = random.nextDouble();
      yoDouble.set(tempDouble);
      dataBufferEntry.setDataAtIndexToYoVariableValue(0);
      dataBufferEntry.copyValueThrough();

      for (int i = 0; i < nPoints; i++)
      {
         assertEquals(tempDouble, dataBufferEntry.getData()[i], 0);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testEnlargeBufferSize()
   {
      Random random = new Random(324270);

      double[] tempData = new double[nPoints];
      int newSizeDelta = random.nextInt(100);

      for (int i = 0; i < nPoints; i++)
      {
         tempData[i] = (double) random.nextInt(20000) / (double) random.nextInt(30);
         yoDouble.set(tempData[i]);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }
      dataBufferEntry.enlargeBufferSize(nPoints + newSizeDelta);

      assertEquals(nPoints + newSizeDelta, dataBufferEntry.getData().length);

      for (int i = 0; i < nPoints; i++)
      {
         assertEquals(tempData[i], dataBufferEntry.getData()[i], 0);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCropData()
   {
      Random random = new Random(2372891);

      double[] tempData = new double[nPoints];

      for (int i = 0; i < nPoints; i++)
      {
         tempData[i] = (double) random.nextInt(20000) / (double) random.nextInt(30);
         yoDouble.set(tempData[i]);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }

      // Test Failure Conditions; data remains unchanged so only examine lengths
      assertEquals(-1, dataBufferEntry.cropData(-1, nPoints));
      assertEquals(nPoints, dataBufferEntry.getData().length);
      assertEquals(-1, dataBufferEntry.cropData(0, nPoints + 1));
      assertEquals(nPoints, dataBufferEntry.getData().length);

      // Test unchanged size
      assertEquals(nPoints, dataBufferEntry.cropData(0, nPoints - 1));
      assertEquals(nPoints, dataBufferEntry.getData().length);

      // Verify data integrity
      for (int i = 0; i < nPoints; i++)
      {
         assertEquals(tempData[i], dataBufferEntry.getData()[i], 0);
      }

      // Test cropping from end     
      assertEquals(nPoints - 100, dataBufferEntry.cropData(0, nPoints - 101));

      // Verify data integrity
      for (int i = 0; i < nPoints - 100; i++)
      {
         assertEquals(tempData[i], dataBufferEntry.getData()[i], 0);
      }

      // Restore dataBufferEntry to original state.     
      dataBufferEntry.enlargeBufferSize(nPoints);
      for (int i = 0; i < nPoints; i++)
      {
         yoDouble.set(tempData[i]);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }

      // Test cropping from beginning
      assertEquals(nPoints - 100, dataBufferEntry.cropData(100, nPoints - 1));

      // Verify data integrity
      for (int i = 0; i < dataBufferEntry.getData().length; i++)
      {
         assertEquals(tempData[100 + i], dataBufferEntry.getData()[i], 0);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCutData()
   {
      Random random = new Random(1230972);

      double[] tempData = new double[nPoints];

      for (int i = 0; i < nPoints; i++)
      {
         tempData[i] = (double) random.nextInt(20000) / (double) random.nextInt(30);
         yoDouble.set(tempData[i]);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }

      // Test Failure Conditions; data remains unchanged so only examine lengths
      assertEquals(-1, dataBufferEntry.cropData(-1, nPoints));
      assertEquals(nPoints, dataBufferEntry.getData().length);
      assertEquals(-1, dataBufferEntry.cropData(0, nPoints + 1));
      assertEquals(nPoints, dataBufferEntry.getData().length);

      // Test unchanged size
      assertEquals(-1, dataBufferEntry.cutData(nPoints/2 + 1, nPoints/2 - 1));
      assertEquals(nPoints, dataBufferEntry.getData().length);

      // Verify data integrity
      for (int i = 0; i < nPoints; i++)
      {
         assertEquals(tempData[i], dataBufferEntry.getData()[i], 0);
      }

      // Test cut one point in the middle:
      int cutPoint = nPoints/2;
      int sizeAfterCut = dataBufferEntry.cutData(cutPoint, cutPoint);
      assertEquals(nPoints-1, sizeAfterCut);
      assertEquals(nPoints-1, dataBufferEntry.getData().length);
      
      // Verify data integrity
      for (int i = 0; i < cutPoint; i++)
      {
         assertEquals(tempData[i], dataBufferEntry.getData()[i], 0);
      }
      
      for (int i = cutPoint; i < nPoints-1; i++)
      {
         assertEquals(tempData[i+1], dataBufferEntry.getData()[i], 0);
      }
      
   // Restore dataBufferEntry to original state.     
      dataBufferEntry.enlargeBufferSize(nPoints);
      for (int i = 0; i < nPoints; i++)
      {
         yoDouble.set(tempData[i]);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }
      
      // Test cutting at beginning   
      sizeAfterCut = dataBufferEntry.cutData(0, 2);
      assertEquals(nPoints - 3, sizeAfterCut);

      // Verify data integrity
      for (int i = 0; i < nPoints-3; i++)
      {
         assertEquals(tempData[i+3], dataBufferEntry.getData()[i], 0);
      }

      // Restore dataBufferEntry to original state.     
      dataBufferEntry.enlargeBufferSize(nPoints);
      for (int i = 0; i < nPoints; i++)
      {
         yoDouble.set(tempData[i]);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }

      // Test cutting at end
      sizeAfterCut = dataBufferEntry.cutData(nPoints - 3, nPoints-1);
      assertEquals(nPoints - 3, sizeAfterCut);

      // Verify data integrity
      for (int i = 0; i < dataBufferEntry.getData().length; i++)
      {
         assertEquals(tempData[i], dataBufferEntry.getData()[i], 0);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testPackData()
   {
      Random random = new Random(4357684);

      double[] tempData = new double[nPoints];
      int newStartIndex = random.nextInt(nPoints - 1);
      for (int i = 0; i < nPoints; i++)
      {
         tempData[i] = (double) random.nextInt(20000) / (double) random.nextInt(30);
         yoDouble.set(tempData[i]);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }

      // Test Bad Start Index, data should be unchanged
      dataBufferEntry.packData(-1);
      for (int i = 0; i < nPoints - 100; i++)
      {
         assertEquals(tempData[i], dataBufferEntry.getData()[i], 0);
      }

      dataBufferEntry.packData(nPoints + 10);
      for (int i = 0; i < nPoints - 100; i++)
      {
         assertEquals(tempData[i], dataBufferEntry.getData()[i], 0);
      }

      // Test packing
      dataBufferEntry.packData(newStartIndex);

      for (int i = 0; i < (nPoints - 1) - newStartIndex; i++)
      {
         assertEquals(tempData[newStartIndex + i], dataBufferEntry.getData()[i], 0);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetMax()
   {
      Random random = new Random(6789423);

      int tempInteger = random.nextInt(500) + 11;
      yoDouble.set(tempInteger);
      dataBufferEntry.setDataAtIndexToYoVariableValue(0);
      yoDouble.set(tempInteger + 10);
      dataBufferEntry.setDataAtIndexToYoVariableValue(1);
      yoDouble.set(tempInteger - 10);
      dataBufferEntry.setDataAtIndexToYoVariableValue(2);
      assertEquals(tempInteger + 10, dataBufferEntry.getMax(), 0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetMin()
   {
      Random random = new Random(213705602);

      int tempInteger = random.nextInt(500) + 11;
      yoDouble.set(tempInteger);
      dataBufferEntry.setDataAtIndexToYoVariableValue(0);
      yoDouble.set(tempInteger + 10);
      dataBufferEntry.setDataAtIndexToYoVariableValue(1);
      yoDouble.set(tempInteger - 10);
      dataBufferEntry.setDataAtIndexToYoVariableValue(2);
      assertEquals(0, dataBufferEntry.getMin(), 0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testMinMaxWithNaN()
   {
      for (int i = 0; i < 100; i++)
      {
         yoDouble.set(Double.NaN);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);         
      }
      assertEquals(0.0, dataBufferEntry.getMin(), 0.0);
      assertEquals(0.0, dataBufferEntry.getMax(), 0.0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testMinMaxWithNaN2()
   {
      Random random = new Random(23785);

      for (int i = 0; i < 100; i++)
      {
         if (i == 50)
            yoDouble.set(Double.NaN);
         else
            yoDouble.set(random.nextDouble());
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }
      assertFalse(Double.isNaN(dataBufferEntry.getMin()));
      assertFalse(Double.isNaN(dataBufferEntry.getMax()));
      assertTrue(dataBufferEntry.getMin() <= dataBufferEntry.getMax());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testResetMinMaxChanged()
   {
      Random random = new Random(90237);

      int tempInteger = random.nextInt(500) + 11;
      yoDouble.set(tempInteger);
      dataBufferEntry.setDataAtIndexToYoVariableValue(0);
      assertTrue(dataBufferEntry.minMaxChanged());
      dataBufferEntry.resetMinMaxChanged();
      assertFalse(dataBufferEntry.minMaxChanged());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetData()
   {
      Random random = new Random(23987);

      double tempDouble = (double) random.nextInt(20000) / (double) random.nextInt(30);
      int randomIndex = random.nextInt(nPoints);
      dataBufferEntry.setData(tempDouble, randomIndex);
      
      assertEquals(tempDouble, dataBufferEntry.getData()[randomIndex], 0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetWindowedData()
   {
      Random random = new Random(37905);

      double tempData[] = new double[nPoints];
      int randomIndex = random.nextInt(nPoints-1);
      for (int i = 0; i < nPoints; i++)
      {
         tempData[i] = (double) random.nextInt(20000) / (double) random.nextInt(30);
         yoDouble.set(tempData[i]);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }
      
      double tempDataSubset[] = new double[nPoints - randomIndex];
      for(int i = 0; i < tempDataSubset.length; i++)
      {
         tempDataSubset[i] = tempData[randomIndex+i];
      }
      
      double windowedData[] = dataBufferEntry.getWindowedData(randomIndex, /*nPoints-1,*/ nPoints - randomIndex);
      
      for(int i = 0; i < windowedData.length; i++)
      {
         assertEquals(tempDataSubset[i], windowedData[i], 0);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testEnableAutoScale()
   {
      dataBufferEntry.enableAutoScale(true);
      assertTrue(dataBufferEntry.isAutoScaleEnabled());
      dataBufferEntry.enableAutoScale(false);
      assertFalse(dataBufferEntry.isAutoScaleEnabled());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetMaxWithParameters()
   {
      Random random = new Random(74839);

      for (int i = 0; i < nPoints; i++)
      {
         yoDouble.set((double) random.nextInt(20000) / (double) random.nextInt(30));
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }
      
      double oldMax = dataBufferEntry.getMax();
      double newMax = oldMax + 100;
      
      dataBufferEntry.setData(newMax, 200);
      dataBufferEntry.setData(oldMax, 400);
      
      assertEquals(newMax, dataBufferEntry.getMax(150,250,150,250), 0);
      assertEquals(newMax, dataBufferEntry.getMax(150,0,150,250), 0);
      assertEquals(newMax, dataBufferEntry.getMax(500,250,150,250), 0);
      
      assertEquals(oldMax, dataBufferEntry.getMax(350,450,350,450), 0);
      assertEquals(oldMax, dataBufferEntry.getMax(350,0,350,450), 0);
      assertEquals(oldMax, dataBufferEntry.getMax(500,450,350,450), 0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetMinWithParameters()
   {
      Random random = new Random(751290);

      for (int i = 0; i < nPoints; i++)
      {
         yoDouble.set((double) random.nextInt(20000) / (double) random.nextInt(30));
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }
      
      double oldMin = dataBufferEntry.getMin();
      double newMin = oldMin - 100;
      
      dataBufferEntry.setData(newMin, 200);
      dataBufferEntry.setData(oldMin, 400);
      
      assertEquals(newMin, dataBufferEntry.getMin(150,250,150,250), 0);
      assertEquals(newMin, dataBufferEntry.getMin(150,0,150,250), 0);
      assertEquals(newMin, dataBufferEntry.getMin(500,250,150,250),0);
      
      assertEquals(oldMin, dataBufferEntry.getMin(350,450,350,450), 0);
      assertEquals(oldMin, dataBufferEntry.getMin(350,0,350,450), 0);
      assertEquals(oldMin, dataBufferEntry.getMin(500,450,350,450), 0);
   }

   @Test
   public void testThinData()
   {
      Random random = new Random(53290);

      for(int i = 0; i < nPoints; i++)
      {
         yoDouble.set((double) random.nextInt(20000) / (double) random.nextInt(30));
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }

      assertTrue(dataBufferEntry.getDataLength() == nPoints);

      int keepEveryNthPoint = 5;
      dataBufferEntry.thinData(keepEveryNthPoint);

      assertTrue(dataBufferEntry.getDataLength() == (nPoints / keepEveryNthPoint));
   }

   @Test
   public void testGetSetInverted()
   {
      dataBufferEntry.setInverted(true);
      assertTrue(dataBufferEntry.getInverted());


      dataBufferEntry.setInverted(false);
      assertFalse(dataBufferEntry.getInverted());
   }

   @Test
   public void testGetVariableName()
   {
      assertTrue(dataBufferEntry.getVariableName().equals(yoDouble.getName()));
   }

   @Test
   public void testGetFullVariableNameWithNameSpace()
   {
      assertTrue(dataBufferEntry.getFullVariableNameWithNameSpace().equals(yoDouble.getFullNameWithNameSpace()));
   }
}
