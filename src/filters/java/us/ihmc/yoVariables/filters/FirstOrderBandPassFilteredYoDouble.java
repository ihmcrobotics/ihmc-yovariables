package us.ihmc.yoVariables.filters;

import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class FirstOrderBandPassFilteredYoDouble  extends FirstOrderFilteredYoDouble
{
   private boolean hasBeenCalled = false;

   private final FirstOrderFilteredYoDouble highPassFilteredInput;

   public FirstOrderBandPassFilteredYoDouble(String name, String description, double minPassThroughFrequency_Hz, double maxPassThroughFrequency_Hz,
                                               YoDouble yoTime, YoRegistry registry)
   {
      super(name, description, maxPassThroughFrequency_Hz, yoTime, FirstOrderFilteredYoDouble.FirstOrderFilterType.LOW_PASS, registry);

      this.highPassFilteredInput = new FirstOrderFilteredYoDouble(name + "HighPassFilteredOnly", description, minPassThroughFrequency_Hz, yoTime,
                                                                    FirstOrderFilteredYoDouble.FirstOrderFilterType.HIGH_PASS, registry);

      setPassBand(minPassThroughFrequency_Hz, maxPassThroughFrequency_Hz);
   }

   public FirstOrderBandPassFilteredYoDouble(String name, String description, double minPassThroughFrequency_Hz, double maxPassThroughFrequency_Hz,
                                               double DT, YoRegistry registry)
   {
      super(name, description, maxPassThroughFrequency_Hz, DT, FirstOrderFilteredYoDouble.FirstOrderFilterType.LOW_PASS, registry);

      this.highPassFilteredInput = new FirstOrderFilteredYoDouble(name + "HighPassFilteredOnly", description, minPassThroughFrequency_Hz, DT,
                                                                    FirstOrderFilteredYoDouble.FirstOrderFilterType.HIGH_PASS, registry);
   }

   private void checkPassband(double minPassThroughFrequency_Hz, double maxPassThroughFrequency_Hz)
   {
      if (minPassThroughFrequency_Hz > maxPassThroughFrequency_Hz)
      {
         throw new RuntimeException("minPassThroughFrequency [ " + minPassThroughFrequency_Hz + " ] > maxPassThroughFrequency [ " + maxPassThroughFrequency_Hz
                                    + " ]");
      }
   }

   public void update(double filterInput)
   {
      if (!hasBeenCalled)
      {
         hasBeenCalled = true;
         this.set(filterInput);
      }
      else
      {
         updateHighPassFilterAndThenLowPassFilterThat(filterInput);
      }
   }

   public void setPassBand(double minPassThroughFreqHz, double maxPassThroughFreqHz)
   {
      checkPassband(minPassThroughFreqHz, maxPassThroughFreqHz);

      highPassFilteredInput.setCutoffFrequencyHz(minPassThroughFreqHz);
      this.setCutoffFrequencyHz(maxPassThroughFreqHz);
   }

   private void updateHighPassFilterAndThenLowPassFilterThat(double filterInput)
   {
      this.highPassFilteredInput.update(filterInput);

      super.update(highPassFilteredInput.getDoubleValue());
   }
}