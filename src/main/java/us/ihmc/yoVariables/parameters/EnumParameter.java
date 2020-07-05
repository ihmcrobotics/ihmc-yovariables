/*
 * Copyright 2017 Florida Institute for Human and Machine Cognition (IHMC) Licensed under the Apache
 * License, Version 2.0 (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License
 * is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
 * or implied. See the License for the specific language governing permissions and limitations under
 * the License.
 */
package us.ihmc.yoVariables.parameters;

import us.ihmc.yoVariables.providers.EnumProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Enum parameter
 *
 * @author Jesper Smith
 */
public class EnumParameter<T extends Enum<T>> extends YoParameter<EnumParameter<T>> implements EnumProvider<T>
{
   private final YoEnum<T> value;
   private final int initialOrdinal;

   /**
    * Create a new Enum parameter, registered to the namespace of the registry.
    *
    * @param name            Desired name. Must be unique in the registry
    * @param registry        YoVariableRegistry to store under
    * @param enumType        The class representing the type of the enum
    * @param allowNullValues Boolean determining if null enum values are permitted
    */
   public EnumParameter(String name, YoRegistry registry, Class<T> enumType, boolean allowNullValues)
   {
      this(name, "", registry, enumType, allowNullValues);
   }

   /**
    * Create a new Enum parameter, registered to the namespace of the registry.
    *
    * @param name            Desired name. Must be unique in the registry
    * @param description     User readable description that describes the purpose of this parameter
    * @param registry        YoVariableRegistry to store under
    * @param enumType        The class representing the type of the enum
    * @param allowNullValues Boolean determining if null enum values are permitted
    */
   public EnumParameter(String name, String description, YoRegistry registry, Class<T> enumType, boolean allowNullValues)
   {
      this(name, description, registry, enumType, allowNullValues, allowNullValues ? null : enumType.getEnumConstants()[0]);
   }

   /**
    * Create a new Enum parameter, registered to the namespace of the registry.
    *
    * @param name            Desired name. Must be unique in the registry
    * @param registry        YoVariableRegistry to store under
    * @param enumType        The class representing the type of the enum
    * @param allowNullValues Boolean determining if null enum values are permitted
    * @param initialValue    Value to set to when no value can be found in the user provided
    *                        parameterLoader
    */
   public EnumParameter(String name, YoRegistry registry, Class<T> enumType, boolean allowNullValues, T initialValue)
   {
      this(name, "", registry, enumType, allowNullValues, initialValue);
   }

   /**
    * Create a new Enum parameter, registered to the namespace of the registry.
    *
    * @param name            Desired name. Must be unique in the registry
    * @param description     User readable description that describes the purpose of this parameter
    * @param registry        YoVariableRegistry to store under
    * @param enumType        The class representing the type of the enum
    * @param allowNullValues Boolean determining if null enum values are permitted
    * @param initialValue    Value to set to when no value can be found in the user provided
    *                        parameterLoader
    */
   public EnumParameter(String name, String description, YoRegistry registry, Class<T> enumType, boolean allowNullValues, T initialValue)
   {
      super(name, description);

      for (T constant : enumType.getEnumConstants())
      {
         if (constant.toString().equalsIgnoreCase("null"))
         {
            throw new RuntimeException("\"" + constant.toString()
                  + "\" is a reserved keyword for EnumParameters. No enum constants named \"null\"(case insensitive) are allowed.");
         }
      }

      this.value = new YoEnumParameter(name, description, registry, enumType, allowNullValues);
      if (!this.value.getAllowNullValue() && initialValue == null)
      {
         throw new RuntimeException("Cannot initialize to null value, allowNullValues is false");
      }

      if (initialValue == null)
      {
         this.initialOrdinal = YoEnum.NULL_VALUE;
      }
      else
      {
         this.initialOrdinal = initialValue.ordinal();
      }

      setSuggestedRange(0, enumType.getEnumConstants().length - 1);
   }

   /**
    * Create a new Enum parameter, registered to the namespace of the registry. This constructor allows
    * a enum parameter based on string values. This should not be used directly by the user, but only
    * internally in the robot data logger.
    *
    * @param name            Desired name. Must be unique in the registry
    * @param description     User readable description that describes the purpose of this parameter
    * @param registry        YoVariableRegistry to store under
    * @param allowNullValues Boolean determining if null enum values are permitted
    * @param constants       Array of enum constants
    */
   public EnumParameter(String name, String description, YoRegistry registry, boolean allowNullValues, String... constants)
   {
      super(name, description);
      for (String constant : constants)
      {
         if (constant == null)
         {
            throw new RuntimeException("One of the enum constants is null.");
         }
         if (constant.equalsIgnoreCase("null"))
         {
            throw new RuntimeException("\"" + constant.toString()
                  + "\" is a reserved keyword for EnumParameters. No enum constants named \"null\"(case insensitive) are allowed.");
         }
      }
      this.value = new YoEnumParameter(name, description, registry, allowNullValues, constants);

      if (!this.value.getAllowNullValue() && constants.length == 0)
      {
         throw new RuntimeException("Cannot initialize an enum parameter with zero elements if allowNullValues is false.");
      }

      if (allowNullValues || constants.length == 0)
      {
         this.initialOrdinal = YoEnum.NULL_VALUE;
      }
      else
      {
         this.initialOrdinal = 0;
      }

      setSuggestedRange(0, constants.length - 1);
   }

   /**
    * Get the current value.
    *
    * @return value for this parameter
    * @throws RuntimeException if the parameter is not loaded yet.
    */
   @Override
   public T getValue()
   {
      checkLoaded();
      return this.value.getEnumValue();
   }

   @Override
   public String getValueAsString()
   {
      return this.value.getStringValue();
   }

   /**
    * Retrieves this EnumParameter's enumType if it is backed by an enum.
    *
    * @return enumType
    * @throws UnsupportedOperationException from {@link YoEnum#checkIfBackedByEnum()}
    */
   public Class<T> getEnumType()
   {
      return value.getEnumType();
   }

   @Override
   YoVariable<?> getVariable()
   {
      return this.value;
   }

   @Override
   void setToString(String valueString)
   {
      if ("null".equalsIgnoreCase(valueString))
      {
         this.value.set(YoEnum.NULL_VALUE);
         return;
      }

      for (int i = 0; i < this.value.getEnumValuesAsString().length; i++)
      {
         if (this.value.getEnumValuesAsString()[i].equals(valueString))
         {
            this.value.set(i);
            return;
         }
      }

      throw new RuntimeException("Cannot set enum value to " + valueString + ", undefined enum constant");
   }

   @Override
   void setToDefault()
   {
      this.value.set(initialOrdinal);
   }

   /**
    * Internal class to set parameter settings for YoEnum
    *
    * @author Jesper Smith
    */
   private class YoEnumParameter extends YoEnum<T>
   {

      public YoEnumParameter(String name, String description, YoRegistry registry, Class<T> enumType, boolean allowNullValues)
      {
         super(name, description, registry, enumType, allowNullValues);
      }

      public YoEnumParameter(String name, String description, YoRegistry registry, boolean allowNullValues, String... values)
      {
         super(name, description, registry, allowNullValues, values);
      }

      @Override
      public boolean isParameter()
      {
         return true;
      }

      @Override
      public YoParameter<?> getParameter()
      {
         return EnumParameter.this;
      }

      @Override
      public YoEnum<T> duplicate(YoRegistry newRegistry)
      {
         EnumParameter<T> newParameter;
         if (isBackedByEnum())
         {
            T initialValue;
            if (initialOrdinal == NULL_VALUE)
            {
               initialValue = null;
            }
            else
            {
               initialValue = getEnumValues()[initialOrdinal];
            }

            newParameter = new EnumParameter<>(getName(), getDescription(), newRegistry, getEnumType(), getAllowNullValue(), initialValue);
         }
         else
         {
            newParameter = new EnumParameter<>(getName(), getDescription(), newRegistry, getAllowNullValue(), getEnumValuesAsString());
         }

         newParameter.value.set(value.getOrdinal());
         newParameter.loadStatus = getLoadStatus();
         return newParameter.value;
      }
   }

}
