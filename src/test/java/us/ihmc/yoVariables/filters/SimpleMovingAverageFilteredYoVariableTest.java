package us.ihmc.yoVariables.filters;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class SimpleMovingAverageFilteredYoVariableTest
{
   @Test
   public void testWithFixedSizeDoubleArrays()
   {
      for (int i = 0; i < 100; i++)
      {
         YoRegistry registry = new YoRegistry("Blop");
         Random random = new Random(6541654L);
         int windowSize = RandomNumbers.nextInt(random, 1, 1000);
         SimpleMovingAverageFilteredYoVariable sma = new SimpleMovingAverageFilteredYoVariable("tested", windowSize, registry);
         double amplitude = 100.0;
         double[] randomArray = RandomNumbers.nextDoubleArray(random, windowSize, amplitude);
         double expected = 0.0;
         for (double val : randomArray)
            expected += val / windowSize;

         for (int j = 0; j < randomArray.length; j++)
         {
            assertFalse(sma.getHasBufferWindowFilled());
            sma.update(randomArray[j]);
         }

         assertTrue(sma.getHasBufferWindowFilled());
         assertEquals(expected, sma.getDoubleValue(), 1.0e-10);
      }
   }

   @Test
   public void testBetaFilteredYoVariable()
   {
      int beta = 5000;
      double pseudoNoise = 0;

      Random random = new Random(1738L);
      YoRegistry registry = new YoRegistry("testRegistry");
      YoDouble positionVariable = new YoDouble("positionVariable", registry);
      SimpleMovingAverageFilteredYoVariable betaFilteredYoVariable = new SimpleMovingAverageFilteredYoVariable("betaFilteredYoVariable",
                                                                                                               beta,
                                                                                                               positionVariable,
                                                                                                               registry);

      positionVariable.set(10);

      for (int i = 0; i < 10000; i++)
      {
         if (i % 2 == 0)
         {
            pseudoNoise = random.nextDouble();
         }
         positionVariable.add(Math.pow(-1, i) * pseudoNoise);
         betaFilteredYoVariable.update();
      }

      assertEquals(10, betaFilteredYoVariable.getDoubleValue(), 1);
   }

   @Test
   public void testTrueMovingAverage()
   {
      int beta = 10;

      YoRegistry registry = new YoRegistry("testRegistry");
      SimpleMovingAverageFilteredYoVariable betaFilteredYoVariable = new SimpleMovingAverageFilteredYoVariable("betaFilteredYoVariable", beta, registry);

      double epsilon = 1e-10;

      betaFilteredYoVariable.update(1.0);
      assertEquals(1.0, betaFilteredYoVariable.getDoubleValue(), epsilon);

      betaFilteredYoVariable.update(2.0);
      assertEquals(1.5, betaFilteredYoVariable.getDoubleValue(), epsilon);

      betaFilteredYoVariable.update(3.0);
      betaFilteredYoVariable.update(4.0);
      betaFilteredYoVariable.update(5.0);
      betaFilteredYoVariable.update(6.0);
      betaFilteredYoVariable.update(7.0);
      betaFilteredYoVariable.update(8.0);
      betaFilteredYoVariable.update(9.0);
      betaFilteredYoVariable.update(10.0);

      assertEquals(5.5, betaFilteredYoVariable.getDoubleValue(), epsilon);
   }
}
