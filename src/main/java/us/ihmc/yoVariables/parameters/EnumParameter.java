/*
 * Copyright 2017 Florida Institute for Human and Machine Cognition (IHMC)
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

import us.ihmc.yoVariables.exceptions.IllegalOperationException;
import us.ihmc.yoVariables.providers.EnumSupplier;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * Enum parameter.
 *
 * @author Jesper Smith
 * @param <E> The enum type used with this parameter.
 * @see YoParameter
 */
public class EnumParameter<E extends Enum<E>> extends YoParameter implements EnumSupplier<E>
{
   /** The variable backing this parameter. */
   private final YoEnum<E> value;
   /** Optional default value used for initializing this parameter. */
   private final int initialOrdinal;

   /**
    * Creates a new enum parameter and registers it to the given registry.
    *
    * @param name           the parameter's name. Must be unique in the registry.
    * @param registry       initial parent registry for this parameter.
    * @param enumType       the class representing the type of the enum.
    * @param allowNullValue whether this variable should support the {@code null} value or not.
    */
   public EnumParameter(String name, YoRegistry registry, Class<E> enumType, boolean allowNullValue)
   {
      this(name, "", registry, enumType, allowNullValue);
   }

   /**
    * Creates a new enum parameter and registers it to the given registry.
    *
    * @param name           the parameter's name. Must be unique in the registry.
    * @param description    description of this parameter's purpose.
    * @param registry       initial parent registry for this parameter.
    * @param enumType       the class representing the type of the enum.
    * @param allowNullValue whether this variable should support the {@code null} value or not.
    */
   public EnumParameter(String name, String description, YoRegistry registry, Class<E> enumType, boolean allowNullValue)
   {
      this(name, description, registry, enumType, allowNullValue, allowNullValue ? null : enumType.getEnumConstants()[0]);
   }

   /**
    * Creates a new enum parameter and registers it to the given registry.
    *
    * @param name           the parameter's name. Must be unique in the registry.
    * @param registry       initial parent registry for this parameter.
    * @param enumType       the class representing the type of the enum.
    * @param allowNullValue whether this variable should support the {@code null} value or not.
    * @param initialValue   value to set to when no value can be found in the user provided parameter
    *                       loader.
    */
   public EnumParameter(String name, YoRegistry registry, Class<E> enumType, boolean allowNullValue, E initialValue)
   {
      this(name, "", registry, enumType, allowNullValue, initialValue);
   }

   /**
    * Creates a new enum parameter and registers it to the given registry.
    *
    * @param name           the parameter's name. Must be unique in the registry.
    * @param description    description of this parameter's purpose.
    * @param registry       initial parent registry for this parameter.
    * @param enumType       the class representing the type of the enum.
    * @param allowNullValue whether this variable should support the {@code null} value or not.
    * @param initialValue   value to set to when no value can be found in the user provided parameter
    *                       loader.
    */
   public EnumParameter(String name, String description, YoRegistry registry, Class<E> enumType, boolean allowNullValue, E initialValue)
   {
      value = new YoEnumParameter(name, description, registry, enumType, allowNullValue);
      if (!value.isNullAllowed() && initialValue == null)
         throw new IllegalArgumentException("Cannot initialize to null value, allowNullValue is false");

      if (initialValue == null)
         this.initialOrdinal = YoEnum.NULL_VALUE;
      else
         this.initialOrdinal = initialValue.ordinal();
   }

   /**
    * Create a new {@code YoEnum} based on the {@code String} representation of each of the enum
    * constants.
    * <p>
    * This constructor is expected to be used only under peculiar circumstances and is not meant for
    * general use. It can be found useful when deserializing data which only allows to retrieve
    * information about the enum constants but cannot provide the actual type of the enum.
    * </p>
    * <p>
    * A {@code EnumParameter} constructed from {@code String}s does not support the part of API
    * interacting with {@code Enum}s.
    * </p>
    *
    * @param name            the parameter's name. Must be unique in the registry.
    * @param description     description of this parameter's purpose.
    * @param registry        initial parent registry for this parameter.
    * @param allowNullValues Boolean determining if null enum values are permitted
    * @param constants       Array of enum constants
    */
   public EnumParameter(String name, String description, YoRegistry registry, boolean allowNullValues, String... constants)
   {
      value = new YoEnumParameter(name, description, registry, allowNullValues, constants);

      if (allowNullValues || constants.length == 0)
         this.initialOrdinal = YoEnum.NULL_VALUE;
      else
         this.initialOrdinal = 0;
   }

   /**
    * Get the current value.
    *
    * @return value for this parameter.
    * @throws IllegalOperationException if the parameter is not loaded yet.
    */
   @Override
   public E get()
   {
      checkLoaded();
      return value.getEnumValue();
   }

   /**
    * Assesses if this parameter is backed by a {@code Class} as an enum type.
    * <p>
    * A {@code EnumParameter} is backed by an enum type when it was constructed directly or indirectly
    * using {@link #EnumParameter(String, String, YoRegistry, Class, boolean, Enum)}.
    * </p>
    * <p>
    * A {@code EnumParameter} is <b>not</b> backed by an enum type when it was constructed using
    * {@link #EnumParameter(String, String, YoRegistry, boolean, String...)}.
    * </p>
    *
    * @return {@code true} if this parameter relies on the actual enum type, {@code false} if it is
    *         only backed by the {@code String} values representing the enum constants.
    */
   public boolean isBackedByEnum()
   {
      return value.isBackedByEnum();
   }

   /**
    * Returns whether this parameter can be set to {@code null} or not.
    *
    * @return {@code true} if the value of {@code null} is allowed with this parameter.
    */
   public boolean isNullAllowed()
   {
      return value.isNullAllowed();
   }

   /**
    * Retrieves this parameter's enum type.
    *
    * @return the type of the enum.
    * @throws UnsupportedOperationException if this parameter is not backed by an enum type.
    * @see #isBackedByEnum()
    */
   public Class<E> getEnumType()
   {
      return value.getEnumType();
   }

   /**
    * Returns all the enum constants as an array.
    *
    * @return the enum constants as an array. Does not contain the {@code null} value regardless of
    *         {@link #isNullAllowed()}.
    * @throws UnsupportedOperationException if this parameter is not backed by an enum type.
    * @see #isBackedByEnum()
    */
   public E[] getEnumValues()
   {
      return value.getEnumValues();
   }

   /**
    * Returns the {@code String} representation for all the enum constants in order as array.
    *
    * @return {@code String} representation for the all enum constants. Does not contain an element for
    *         representing the {@code null} value regardless of {@link #isNullAllowed()}.
    */
   public String[] getEnumValuesAsString()
   {
      return value.getEnumValuesAsString();
   }

   /**
    * Gets the number of enum values declared for this {@code EnumParameter}.
    *
    * @return number of constants for the enum backing this parameter.
    */
   public int getEnumSize()
   {
      return value.getEnumSize();
   }

   /** {@inheritDoc} */
   @Override
   YoVariable getVariable()
   {
      return value;
   }

   /** {@inheritDoc} */
   @Override
   void setToDefault()
   {
      value.set(initialOrdinal);
   }

   /**
    * Internal class to set parameter settings for {@code YoEnum}.
    *
    * @author Jesper Smith
    */
   private class YoEnumParameter extends YoEnum<E>
   {

      public YoEnumParameter(String name, String description, YoRegistry registry, Class<E> enumType, boolean allowNullValues)
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
      public EnumParameter<E> getParameter()
      {
         return EnumParameter.this;
      }

      @Override
      public YoEnum<E> duplicate(YoRegistry newRegistry)
      {
         EnumParameter<E> newParameter;
         if (isBackedByEnum())
         {
            E initialValue;
            if (initialOrdinal == NULL_VALUE)
            {
               initialValue = null;
            }
            else
            {
               initialValue = getEnumValues()[initialOrdinal];
            }

            newParameter = new EnumParameter<>(getName(), getDescription(), newRegistry, getEnumType(), isNullAllowed(), initialValue);
         }
         else
         {
            newParameter = new EnumParameter<>(getName(), getDescription(), newRegistry, isNullAllowed(), getEnumValuesAsString());
         }

         newParameter.value.set(value.getOrdinal());
         newParameter.loadStatus = getLoadStatus();
         return newParameter.value;
      }
   }
}
