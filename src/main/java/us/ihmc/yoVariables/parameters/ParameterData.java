/*
 * Copyright 2020 Florida Institute for Human and Machine Cognition (IHMC)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package us.ihmc.yoVariables.parameters;

import us.ihmc.euclid.tools.EuclidHashCodeTools;

/**
 * Minimalist data structure used in parameter reader.
 */
public class ParameterData
{
   private final String value;
   private final double min;
   private final double max;

   /**
    * Creates and initializes a new parameter data.
    * 
    * @param value the {@code String} representation of the parameter's value.
    * @param min   the lower bound associated to the parameter.
    * @param max   the upper bound associated to the parameter.
    */
   public ParameterData(String value, String min, String max)
   {
      this.value = value;
      if (min != null && max != null)
      {
         this.min = Double.parseDouble(min);
         this.max = Double.parseDouble(max);
      }
      else
      {
         this.min = 0.0;
         this.max = 1.0;
      }
   }

   /**
    * Creates and initializes a new parameter data.
    * <p>
    * Initializes the min and max field to {@code 0} and {@code 1} respectively.
    * </p>
    * 
    * @param value the {@code String} representation of the parameter's value.
    */
   public ParameterData(String value)
   {
      this.value = value;
      min = 0.0;
      max = 1.0;
   }

   /**
    * Initializes the given parameter with the data in {@code this}.
    * 
    * @param parameter the parameter to initialize. Modified.
    */
   public void setParameterFromThis(YoParameter parameter)
   {
      parameter.load(value);
      parameter.setParameterBounds(min, max);
   }

   /**
    * Returns the parameter value as {@code String}.
    * 
    * @return the parameter value as {@code String}.
    */
   public String getValue()
   {
      return value;
   }

   /**
    * Returns the parameter min value if set or {@code 0.0} otherwise.
    * 
    * @return the parameter min value if set or {@code 0.0} otherwise.
    */
   public double getMin()
   {
      return min;
   }

   /**
    * Returns the parameter max value if set or {@code 1.0} otherwise.
    * 
    * @return the parameter max value if set or {@code 1.0} otherwise.
    */
   public double getMax()
   {
      return max;
   }

   @Override
   public int hashCode()
   {
      long bits = EuclidHashCodeTools.addToHashCode(1L, value);
      bits = EuclidHashCodeTools.addToHashCode(bits, min);
      bits = EuclidHashCodeTools.addToHashCode(bits, max);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof ParameterData)
      {
         ParameterData other = (ParameterData) object;
         return value.equals(other.value) && min == other.min && max == other.max;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      return String.format("[value=%s, min=%s, max=%s]", value, Double.toString(min), Double.toString(max));
   }
}
