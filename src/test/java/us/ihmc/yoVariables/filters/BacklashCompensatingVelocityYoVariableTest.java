package us.ihmc.yoVariables.filters;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class BacklashCompensatingVelocityYoVariableTest
{
   private static final double EPSILON = 1e-8;

	@Test
   public void testWithoutBacklashOrFiltering1()
   {
      Random rand = new Random(1798L);

      YoRegistry registry = new YoRegistry("blop");
      YoDouble alphaVariable = new YoDouble("alpha", registry);
      double dt = RandomNumbers.nextDouble(rand, 1e-8, 1.0);
      YoDouble slopTime = new YoDouble("slop", registry);
      BacklashCompensatingVelocityYoVariable unprocessed = new BacklashCompensatingVelocityYoVariable("", "", alphaVariable, dt, slopTime, registry);

      double rawPosition = 0.0, rawPositionPrevValue = 0.0;
      unprocessed.update(rawPosition);

      for (int i = 0; i < 1000; i++)
      {
         rawPosition = RandomNumbers.nextDouble(rand, -100.0, 100.0);
         unprocessed.update(rawPosition);

         double rawVelocity = (rawPosition - rawPositionPrevValue) / dt;

         assertEquals(rawVelocity, unprocessed.getDoubleValue(), EPSILON);

         rawPositionPrevValue = rawPosition;
      }
   }

	@Test
   public void testWithoutBacklashOrFiltering2()
   {
      Random rand = new Random(1798L);

      YoRegistry registry = new YoRegistry("blop");
      YoDouble alphaVariable = new YoDouble("alpha", registry);
      double dt = RandomNumbers.nextDouble(rand, 1e-8, 1.0);
      YoDouble slopTime = new YoDouble("slop", registry);
      YoDouble rawPosition = new YoDouble("rawPosition", registry);
      BacklashCompensatingVelocityYoVariable unprocessed = new BacklashCompensatingVelocityYoVariable("", "", alphaVariable, rawPosition, dt, slopTime,
                                                                                                      registry);

      double rawPositionPrevValue = 0.0;
      unprocessed.update();

      for (int i = 0; i < 1000; i++)
      {
         rawPosition.set(RandomNumbers.nextDouble(rand, -100.0, 100.0));
         unprocessed.update();

         double rawVelocity = (rawPosition.getDoubleValue() - rawPositionPrevValue) / dt;

         assertEquals(rawVelocity, unprocessed.getDoubleValue(), EPSILON);

         rawPositionPrevValue = rawPosition.getDoubleValue();
      }
   }

	@Test
   public void testWithoutBacklash1()
   {
      Random rand = new Random(1798L);

      YoRegistry registry = new YoRegistry("blop");
      YoDouble alphaVariable = new YoDouble("alpha", registry);
      alphaVariable.set(RandomNumbers.nextDouble(rand, 0.1, 1.0));
      double dt = RandomNumbers.nextDouble(rand, 1e-8, 1.0);
      YoDouble slopTime = new YoDouble("slop", registry);
      YoDouble rawPosition = new YoDouble("rawPosition", registry);
      FilteredFiniteDifferenceYoVariable filtVelocity = new FilteredFiniteDifferenceYoVariable("filtVelocity", "", alphaVariable, rawPosition, dt, registry);
      BacklashCompensatingVelocityYoVariable filteredOnly = new BacklashCompensatingVelocityYoVariable("", "", alphaVariable, dt, slopTime, registry);

      filtVelocity.update();
      filteredOnly.update(rawPosition.getDoubleValue());

      for (int i = 0; i < 1000; i++)
      {
         alphaVariable.set(RandomNumbers.nextDouble(rand, 0.1, 1.0));
         rawPosition.set(RandomNumbers.nextDouble(rand, -100.0, 100.0));
         filtVelocity.update();
         filteredOnly.update(rawPosition.getDoubleValue());

         assertEquals(filtVelocity.getDoubleValue(), filteredOnly.getDoubleValue(), EPSILON);
      }
   }

	@Test
   public void testWithoutBacklash2()
   {
      Random rand = new Random(1798L);

      YoRegistry registry = new YoRegistry("blop");
      YoDouble alphaVariable = new YoDouble("alpha", registry);
      alphaVariable.set(RandomNumbers.nextDouble(rand, 0.0, 1.0));
      double dt = RandomNumbers.nextDouble(rand, 1e-8, 1.0);
      YoDouble slopTime = new YoDouble("slop", registry);
      YoDouble rawPosition = new YoDouble("rawPosition", registry);
      FilteredFiniteDifferenceYoVariable filtVelocity = new FilteredFiniteDifferenceYoVariable("filtVelocity", "", alphaVariable, rawPosition, dt, registry);
      BacklashCompensatingVelocityYoVariable filteredOnly = new BacklashCompensatingVelocityYoVariable("", "", alphaVariable, rawPosition, dt, slopTime,
                                                                                                       registry);

      filtVelocity.update();
      filteredOnly.update();

      for (int i = 0; i < 1000; i++)
      {
         alphaVariable.set(RandomNumbers.nextDouble(rand, 0.1, 1.0));
         rawPosition.set(RandomNumbers.nextDouble(rand, -100.0, 100.0));
         filtVelocity.update();
         filteredOnly.update();

         assertEquals(filtVelocity.getDoubleValue(), filteredOnly.getDoubleValue(), EPSILON);
      }
   }

	@Test
   public void testVelocityPositiveWithoutCrossingZero2()
   {
      Random rand = new Random(1798L);

      YoRegistry registry = new YoRegistry("blop");
      YoDouble alphaVariable = new YoDouble("alpha", registry);
      alphaVariable.set(RandomNumbers.nextDouble(rand, 0.0, 1.0));
      double dt = RandomNumbers.nextDouble(rand, 1e-8, 1.0);
      YoDouble slopTime = new YoDouble("slop", registry);
      YoDouble rawPosition = new YoDouble("rawPosition", registry);
      FilteredFiniteDifferenceYoVariable filtVelocity = new FilteredFiniteDifferenceYoVariable("filtVelocity", "", alphaVariable, rawPosition, dt, registry);
      BacklashCompensatingVelocityYoVariable backlashAndFiltered = new BacklashCompensatingVelocityYoVariable("", "", alphaVariable, rawPosition, dt, slopTime,
                                                                                                              registry);

      filtVelocity.update();
      backlashAndFiltered.update();

      // In this test, the position is only increasing, so there should be no backlash filtering that gets applied.

      double currentTime = 0.0;
      for (int i = 0; i < 10000; i++)
      {
         slopTime.set(RandomNumbers.nextDouble(rand, 0.0, 10.0));
         alphaVariable.set(RandomNumbers.nextDouble(rand, 0.1, 1.0));
         rawPosition.add(RandomNumbers.nextDouble(rand, 0.0, 101.0));
         filtVelocity.update();
         backlashAndFiltered.update();

//         if (currentTime > 2.0 * slopTime.getDoubleValue())
            assertEquals(filtVelocity.getDoubleValue(), backlashAndFiltered.getDoubleValue(), EPSILON);

         currentTime += dt;
      }
   }

	@Test
   public void testVelocityNegativeWithoutCrossingZero2()
   {
      Random rand = new Random(1798L);

      YoRegistry registry = new YoRegistry("blop");
      YoDouble alphaVariable = new YoDouble("alpha", registry);
      alphaVariable.set(RandomNumbers.nextDouble(rand, 0.0, 1.0));
      double dt = RandomNumbers.nextDouble(rand, 1e-8, 1.0);
      YoDouble slopTime = new YoDouble("slop", registry);
      YoDouble rawPosition = new YoDouble("rawPosition", registry);
      FilteredFiniteDifferenceYoVariable filtVelocity = new FilteredFiniteDifferenceYoVariable("filtVelocity", "", alphaVariable, rawPosition, dt, registry);
      BacklashCompensatingVelocityYoVariable backlashAndFiltered = new BacklashCompensatingVelocityYoVariable("", "", alphaVariable, rawPosition, dt, slopTime,
                                                                                                              registry);

      filtVelocity.update();
      backlashAndFiltered.update();

      for (int i = 0; i < 1000; i++)
      {
         slopTime.set(RandomNumbers.nextDouble(rand, 0.0, 100.0));
         alphaVariable.set(RandomNumbers.nextDouble(rand, 0.0, 1.0));
         rawPosition.sub(RandomNumbers.nextDouble(rand, 0.0, 101.0));
         filtVelocity.update();
         backlashAndFiltered.update();

         assertEquals(filtVelocity.getDoubleValue(), backlashAndFiltered.getDoubleValue(), EPSILON);
      }
   }


	
   @Test
   public void testNoisySignalAndMakeSureVelocityHasSignalContent()
   {
      Random random = new Random(1798L);

      YoRegistry registry = new YoRegistry("Registry");
      YoDouble alphaVariable = new YoDouble("alpha", registry);
      YoDouble slopTime = new YoDouble("slopTime", registry);
      YoDouble cleanPosition = new YoDouble("cleanPosition", registry);
      YoDouble noisyPosition = new YoDouble("noisyPosition", registry);
      YoDouble cleanVelocity = new YoDouble("cleanVelocity", registry);

      
      YoDouble reconstructedPosition = new YoDouble("reconstructedPosition", registry);
      YoDouble reconstructedPosition2 = new YoDouble("reconstructedPosition2", registry);

      YoDouble totalReconstructedPositionError2 = new YoDouble("totalReconstructedPositionError2", registry);
      
      YoDouble averageReconstructedPositionError2 = new YoDouble("averageReconstructedPositionError2", registry);

      double dt = 0.001;
      double totalTime = 5.0;

      double amplitude = 2.0;
      double frequency = 1.0;
      double noiseAmplitude = 0.01;
      
      slopTime.set(0.1);
      alphaVariable.set(0.95);
      
      BacklashCompensatingVelocityYoVariable revisedBacklashCompensatingVelocity = new BacklashCompensatingVelocityYoVariable("bl_qd_velocity2", "", alphaVariable, noisyPosition, dt, slopTime, registry);

      reconstructedPosition2.set(amplitude);
      
//      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Test"));
//      scs.addYoVariableRegistry(registry);
//      scs.startOnAThread();
      
      for (double time = 0.0; time < totalTime; time = time + dt)
      {
         cleanPosition.set(amplitude * Math.cos(2.0 * Math.PI * frequency * time)); 
         cleanVelocity.set(-2.0 * Math.PI * amplitude * frequency * Math.sin(2.0 * Math.PI * frequency * time));
         
         noisyPosition.set(cleanPosition.getDoubleValue());
         noisyPosition.add(RandomNumbers.nextDouble(random, noiseAmplitude));
         
         revisedBacklashCompensatingVelocity.update();
         
         reconstructedPosition2.add(revisedBacklashCompensatingVelocity.getDoubleValue() * dt);
         
        double positionError2 = reconstructedPosition2.getDoubleValue() - cleanPosition.getDoubleValue();
        totalReconstructedPositionError2.add(Math.abs(positionError2) * dt);
                
//        scs.tickAndUpdate();
      }
      
      averageReconstructedPositionError2.set(totalReconstructedPositionError2.getDoubleValue() / totalTime);
      
      // The original one doesn't do very well with noisy signals because it thinks the noise is backlash.
      assertTrue(averageReconstructedPositionError2.getDoubleValue() < 0.25);
   }
   
   @Test
   public void testSignalWithBacklash()
   {
      YoRegistry registry = new YoRegistry("Registry");
      YoDouble alphaVariable = new YoDouble("alpha", registry);
      YoDouble slopTime = new YoDouble("slopTime", registry);
      
      YoDouble cleanPosition = new YoDouble("cleanPosition", registry);
      YoDouble backlashyPosition = new YoDouble("backlashyPosition", registry);
      YoDouble cleanVelocity = new YoDouble("cleanVelocity", registry);

      
      YoDouble reconstructedPosition2 = new YoDouble("reconstructedPosition2", registry);

      YoDouble totalReconstructedPositionError2 = new YoDouble("totalReconstructedPositionError2", registry);
      
      YoDouble averageReconstructedPositionError2 = new YoDouble("averageReconstructedPositionError2", registry);

      double dt = 0.001;
      double totalTime = 5.0;

      double amplitude = 2.0;
      double frequency = 1.0;
      double backlashAmount = 0.1;
      
      slopTime.set(0.1);
      alphaVariable.set(0.95);
      
      BacklashCompensatingVelocityYoVariable revisedBacklashCompensatingVelocity = new BacklashCompensatingVelocityYoVariable("bl_qd_velocity2", "", alphaVariable, backlashyPosition, dt, slopTime, registry);

      reconstructedPosition2.set(amplitude);
      
//      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Test"));
//      scs.addYoVariableRegistry(registry);
//      scs.startOnAThread();
      
      for (double time = 0.0; time < totalTime; time = time + dt)
      {
         cleanPosition.set(amplitude * Math.cos(2.0 * Math.PI * frequency * time)); 
         cleanVelocity.set(-2.0 * Math.PI * amplitude * frequency * Math.sin(2.0 * Math.PI * frequency * time));
         
         backlashyPosition.set(cleanPosition.getDoubleValue());
         if(cleanVelocity.getDoubleValue() > 0.0)
         {
            backlashyPosition.add(backlashAmount);
         }
         
         revisedBacklashCompensatingVelocity.update();
         
         reconstructedPosition2.add(revisedBacklashCompensatingVelocity.getDoubleValue() * dt);
         
        double positionError2 = reconstructedPosition2.getDoubleValue() - cleanPosition.getDoubleValue();
        totalReconstructedPositionError2.add(Math.abs(positionError2) * dt);
                
//        scs.tickAndUpdate();
      }
      
      averageReconstructedPositionError2.set(totalReconstructedPositionError2.getDoubleValue() / totalTime);
      
      assertTrue(averageReconstructedPositionError2.getDoubleValue() < 0.25);
   }
   
   
   @Test
   public void testRemoveSquareWaveBacklash()
   {
      YoRegistry registry = new YoRegistry("Registry");
      YoDouble alphaVariable = new YoDouble("alpha", registry);
      YoDouble slopTime = new YoDouble("slopTime", registry);
      
      YoDouble backlashyPosition = new YoDouble("backlashyPosition", registry);

      double dt = 0.001;
      double totalTime = 5.0;

      double frequency = 30.0;
      double backlashAmount = 0.1;
      
      slopTime.set(0.1);
      alphaVariable.set(0.95);
      
      BacklashCompensatingVelocityYoVariable revisedBacklashCompensatingVelocity = new BacklashCompensatingVelocityYoVariable("bl_qd_velocity2", "", alphaVariable, backlashyPosition, dt, slopTime, registry);

//      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Test"));
//      scs.addYoVariableRegistry(registry);
//      scs.startOnAThread();

      // initialize the system to make sure it's resting up against one of the heads of slop. Previously without this, it was a lucky test.
      backlashyPosition.set(0.0);
      revisedBacklashCompensatingVelocity.update();
      backlashyPosition.set(backlashAmount);
      for (int i = 0; i < 2.0 * slopTime.getDoubleValue() / dt; i++)
         revisedBacklashCompensatingVelocity.update();


      for (double time = 0.0; time < totalTime; time = time + dt)
      {
         backlashyPosition.set(Math.cos(2.0 * Math.PI * frequency * time)); 
         if (backlashyPosition.getDoubleValue() > 0.0) 
         {
            backlashyPosition.set(backlashAmount);
         }
         else
         {
            backlashyPosition.set(-backlashAmount);
         }
         
         revisedBacklashCompensatingVelocity.update();
         
         assertEquals(0.0, revisedBacklashCompensatingVelocity.getDoubleValue(), 1e-3);
                
//        scs.tickAndUpdate();
      }
   }
}
