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
package us.ihmc.yoVariables.variable;

import java.util.Arrays;

import us.ihmc.yoVariables.providers.EnumProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Enum implementation of a {@code YoVariable}.
 * 
 * @param <E> The enum type used with this variable.
 * @see YoVariable
 */
public class YoEnum<E extends Enum<E>> extends YoVariable implements EnumProvider<E>
{
   /** Ordinal representing the {@code null} value. */
   public static final int NULL_VALUE = -1;
   /** String representation of the {@code null} value. */
   public static final String NULL_VALUE_STRING = "null";

   private final Class<E> enumType;
   private final boolean allowNullValue;
   private final E[] enumValues;
   private final String[] enumValuesAsString;

   /** The ordinal used to track the current enum value for this variable. */
   private int valueOrdinal;

   /**
    * Create a new {@code YoEnum} and initializes to the first enum constant.
    *
    * @param name     the name for this variable that can be used to retrieve it from a
    *                 {@link YoRegistry}.
    * @param registry initial parent registry for this variable.
    * @param enumType the class representing the type of the enum.
    */
   public YoEnum(String name, YoRegistry registry, Class<E> enumType)
   {
      this(name, "", registry, enumType, false);
   }

   /**
    * Create a new {@code YoEnum} and initializes to the first enum constant.
    *
    * @param name           the name for this variable that can be used to retrieve it from a
    *                       {@link YoRegistry}.
    * @param registry       initial parent registry for this variable.
    * @param enumType       the class representing the type of the enum.
    * @param allowNullValue whether this variable should support the {@code null} value or not.
    */
   public YoEnum(String name, YoRegistry registry, Class<E> enumType, boolean allowNullValue)
   {
      this(name, "", registry, enumType, allowNullValue);
   }

   /**
    * Create a new {@code YoEnum} and initializes to the first enum constant or {@code null} if
    * allowed.
    *
    * @param name           the name for this variable that can be used to retrieve it from a
    *                       {@link YoRegistry}.
    * @param description    description of this variable's purpose.
    * @param registry       initial parent registry for this variable.
    * @param enumType       the class representing the type of the enum.
    * @param allowNullValue whether this variable should support the {@code null} value or not.
    */
   public YoEnum(String name, String description, YoRegistry registry, Class<E> enumType, boolean allowNullValue)
   {
      super(YoVariableType.ENUM, name, description, registry);

      this.enumType = enumType;
      this.allowNullValue = allowNullValue;
      this.enumValues = enumType.getEnumConstants();

      enumValuesAsString = new String[enumValues.length];

      for (int i = 0; i < enumValues.length; i++)
      {
         String enumValueAsString = enumValues[i].toString();

         if (enumValueAsString.equalsIgnoreCase(NULL_VALUE_STRING))
            throw new IllegalArgumentException(enumValueAsString + " is a restricted keyword. No enum constants named \"null\"(case insensitive) are allowed.");

         enumValuesAsString[i] = enumValueAsString;
      }

      if (!allowNullValue && enumValues.length == 0)
         throw new IllegalArgumentException("Cannot initialize an enum variable with zero elements if allowNullValue is false.");

      if (allowNullValue || enumValues.length == 0)
         set(NULL_VALUE);
      else
         set(0);

      setVariableBounds(allowNullValue ? NULL_VALUE : 0, enumValues.length - 1);
   }

   /**
    * Create a new {@code YoEnum} based on the {@code String} representation of each of the enum
    * constants and initializes to the first enum constant or {@code null} if allowed.
    * <p>
    * This constructor is expected to be used only under peculiar circumstances and is not meant for
    * general use. It can be found useful when deserializing data which only allows to retrieve
    * information about the enum constants but cannot provide the actual type of the enum.
    * </p>
    * <p>
    * A {@code YoEnum} constructed from {@code String}s does not support the part of API interacting
    * with {@code Enum}s.
    * </p>
    *
    * @param name           the name for this variable that can be used to retrieve it from a
    *                       {@link YoRegistry}.
    * @param description    description of this variable's purpose.
    * @param registry       initial parent registry for this variable.
    * @param allowNullValue whether this variable should support the {@code null} value or not.
    * @param constants      the {@code String}s for each constant, in order, for the enum this variable
    *                       represents.
    * @see YoVariable#YoVariable(YoVariableType, String, String, YoRegistry)
    */
   public YoEnum(String name, String description, YoRegistry registry, boolean allowNullValue, String... constants)
   {
      super(YoVariableType.ENUM, name, description, registry);

      this.enumType = null;
      this.allowNullValue = allowNullValue;
      this.enumValues = null;

      for (String constant : constants)
      {
         if (constant == null)
            throw new IllegalArgumentException("One of the enum constants is null.");
         if (constant.equalsIgnoreCase(NULL_VALUE_STRING))
            throw new IllegalArgumentException(constant + " is a restricted keyword. No enum constants named \"null\"(case insensitive) are allowed.");
      }

      enumValuesAsString = Arrays.copyOf(constants, constants.length);

      if (!allowNullValue && constants.length == 0)
         throw new IllegalArgumentException("Cannot initialize an enum variable with zero elements if allowNullValue is false.");

      if (allowNullValue || constants.length == 0)
         set(NULL_VALUE);
      else
         set(0);

      setVariableBounds(allowNullValue ? NULL_VALUE : 0, constants.length - 1);
   }

   /**
    * Assesses if this variable is backed by a {@code Class} as an enum type.
    * <p>
    * A {@code YoEnum} is backed by an enum type when it was constructed directly or indirectly using
    * {@link #YoEnum(String, String, YoRegistry, Class, boolean)}.
    * </p>
    * <p>
    * A {@code YoEnum} is <b>not</b> backed by an enum type when it was constructed using
    * {@link #YoEnum(String, String, YoRegistry, boolean, String...)}.
    * </p>
    *
    * @return {@code true} if this variable relies on the actual enum type, {@code false} if it is only
    *         backed by the {@code String} values representing the enum constants.
    */
   public boolean isBackedByEnum()
   {
      return enumType != null;
   }

   /**
    * Returns whether this variable can be set to {@code null} or not.
    *
    * @return {@code true} if the value of {@code null} is allowed with this variable.
    */
   public boolean isNullAllowed()
   {
      return allowNullValue;
   }

   /**
    * Essentially performs {@link #isBackedByEnum()} and throws an exception if {@code false}.
    *
    * @throws UnsupportedOperationException if not backed by a {@code Class} as an enum type.
    */
   private void checkIfBackedByEnum()
   {
      if (enumType == null)
      {
         throw new UnsupportedOperationException("This YoEnum is not backed by an Enum type.");
      }
   }

   /**
    * Tests if the variable's current value is equal to the given integer.
    *
    * @param value the query.
    * @return boolean if this variable's value is equal to the query.
    * @throws UnsupportedOperationException if this variable is not backed by an enum type.
    * @see #isBackedByEnum()
    */
   public boolean valueEquals(E value)
   {
      checkIfBackedByEnum();
      if (valueOrdinal == NULL_VALUE)
         return value == null;

      return value.ordinal() == valueOrdinal;
   }

   /**
    * Retrieves this variable's enum type.
    *
    * @return the type of the enum.
    * @throws UnsupportedOperationException if this variable is not backed by an enum type.
    * @see #isBackedByEnum()
    */
   public Class<E> getEnumType()
   {
      checkIfBackedByEnum();
      return enumType;
   }

   /**
    * Returns all the enum constants as an array.
    *
    * @return the enum constants as an array. Does not contain the {@code null} value regardless of
    *         {@link #isNullAllowed()}.
    * @throws UnsupportedOperationException if this variable is not backed by an enum type.
    * @see #isBackedByEnum()
    */
   public E[] getEnumValues()
   {
      checkIfBackedByEnum();
      return enumValues;
   }

   /**
    * Returns the {@code String} representation for all the enum constants in order as array.
    *
    * @return {@code String} representation for the all enum constants. Does not contain an element for
    *         representing the {@code null} value regardless of {@link #isNullAllowed()}.
    */
   public String[] getEnumValuesAsString()
   {
      return enumValuesAsString;
   }

   /**
    * Retrieves the current enum value of this variable.
    *
    * @return the internal enum value of this variable.
    * @throws UnsupportedOperationException if this variable is not backed by an enum type.
    * @see #isBackedByEnum()
    */
   @Override
   public E getValue()
   {
      return getEnumValue();
   }

   /**
    * Retrieves the current enum value of this variable.
    *
    * @return the internal enum value of this variable.
    * @throws UnsupportedOperationException if this variable is not backed by an enum type.
    * @see #isBackedByEnum()
    */
   public E getEnumValue()
   {
      checkIfBackedByEnum();
      return valueOrdinal == NULL_VALUE ? null : enumValues[valueOrdinal];
   }

   /**
    * Sets this variable's current enum value.
    * <p>
    * This variable's listeners will be notified if this variable's value is changed.
    * </p>
    *
    * @param value the new enum value for this variable.
    * @return {@code true} if this variable's value changed, {@code false} otherwise.
    * @throws UnsupportedOperationException if this variable is not backed by an enum type.
    * @throws IllegalArgumentException      if {@code value == null} and {@link #isNullAllowed() ==
    *                                       false}.
    * @see #isBackedByEnum()
    */
   public boolean set(E value)
   {
      return set(value, true);
   }

   /**
    * Sets this variable's current enum value.
    * <p>
    * This variable's listeners will be notified if this variable's value is changed.
    * </p>
    *
    * @param value           the new enum value for this variable.
    * @param notifyListeners whether to notify this variable's listeners if this operation results in
    *                        changing this variable's current value.
    * @return {@code true} if this variable's value changed, {@code false} otherwise.
    * @throws UnsupportedOperationException if this variable is not backed by an enum type.
    * @throws IllegalArgumentException      if {@code value == null} and {@code null} is not allowed.
    * @see #isBackedByEnum()
    * @see #YoEnum(String, YoRegistry, Class, boolean)
    */
   public boolean set(E value, boolean notifyListeners)
   {
      checkIfBackedByEnum();

      if (!allowNullValue && value == null)
      {
         throw new IllegalArgumentException("Setting YoEnum " + getName()
               + " to null. Must set allowNullValue to true in the constructor if you ever want to set it to null.");
      }

      return set(value == null ? NULL_VALUE : value.ordinal(), notifyListeners);
   }

   /**
    * Retrieves the ordinal representing the current enum value of this variable.
    *
    * @return the internal enum value as its ordinal.
    */
   public int getOrdinal()
   {
      return valueOrdinal;
   }

   /**
    * Sets this variable's current value by setting the ordinal.
    * <p>
    * This variable's listeners will be notified if this variable's value is changed.
    * </p>
    *
    * @param ordinal the new enum value for this variable.
    * @return {@code true} if this variable's value changed, {@code false} otherwise.
    * @throws UnsupportedOperationException if this variable is not backed by an enum type.
    * @throws IllegalArgumentException      if {@code value == null} and {@link #isNullAllowed() ==
    *                                       false}.
    * @see #isBackedByEnum()
    */
   public boolean set(int ordinal)
   {
      return set(ordinal, true);
   }

   /**
    * Sets the YoEnum to the given ordinal.
    *
    * @param ordinal         integer ordinal for the enum value to set this YoEnum to
    * @param notifyListeners boolean determining whether or not to call {@link #notifyListeners()}
    * @return boolean if given value is valid and YoEnum is set or the same
    * @throws RuntimeException if ordinal falls out of allowed range for this YoEnum
    */
   public boolean set(int ordinal, boolean notifyListeners)
   {
      checkBounds(ordinal);

      if (valueOrdinal != NULL_VALUE)
      {
         if (!(valueOrdinal == ordinal))
         {
            valueOrdinal = ordinal;
            if (notifyListeners)
            {
               notifyListeners();
            }
            return true;
         }
      }
      else
      {
         valueOrdinal = ordinal;
         if (notifyListeners)
         {
            notifyListeners();
         }
         return true;
      }

      return false;

   }

   /**
    * Exception-throwing check if the given value falls within the allowed range for this YoEnum.
    *
    * @param ordinal integer ordinal value to check
    * @throws RuntimeException if the given value is null and null is not allowed for this YoEnum or if
    *                          the value falls out of the allowed range
    */
   private void checkBounds(int ordinal)
   {
      if (ordinal < 0 && !(allowNullValue && ordinal == NULL_VALUE) || ordinal >= enumValuesAsString.length)
      {
         throw new RuntimeException("Enum constant associated with value " + ordinal + " not present. VariableName = " + getFullNameString());
      }
   }

   /**
    * Retrieve the String representing this YoEnum.
    *
    * @return String from {@link #enumValuesAsString} for this YoEnum's internal enum state
    */
   public String getStringValue()
   {
      if (valueOrdinal == NULL_VALUE)
      {
         return NULL_VALUE_STRING;
      }
      else
      {
         return enumValuesAsString[valueOrdinal];
      }
   }

   /**
    * Set the value of this YoEnum using the given double, interpreted as a rounded integer ordinal for
    * an enum value.
    *
    * @param value           double to convert and set this YoEnum to
    * @param notifyListeners boolean determining whether or not to call {@link #notifyListeners()}
    */
   @Override
   public boolean setValueFromDouble(double value, boolean notifyListeners)
   {
      int ordinal = (int) Math.round(value);
      ordinal = Math.min(ordinal, getEnumSize() - 1);
      ordinal = Math.max(ordinal, allowNullValue ? NULL_VALUE : 0);
      return set(ordinal, notifyListeners);
   }

   /**
    * Retrieves this YoEnum's value as a double.
    *
    * @return return-casted double value of this YoEnum's internal enum value
    */
   @Override
   public double getValueAsDouble()
   {
      return valueOrdinal;
   }

   /**
    * Retrieves this YoEnum's value as a long.
    *
    * @return return-casted long value of this YoEnum's internal enum value
    */
   @Override
   public long getValueAsLongBits()
   {
      return valueOrdinal;
   }

   /**
    * Sets the internal enum value of this YoEnum using the static integer enum ordinal cast of the
    * passed long value.
    *
    * @param value           long to set this variable's value to
    * @param notifyListeners boolean determining whether or not to call {@link #notifyListeners()}
    */
   @Override
   public boolean setValueFromLongBits(long value, boolean notifyListeners)
   {
      return set((int) value, notifyListeners);
   }

   /**
    * Sets this variable's value from the other variable once casted to {@code YoEnum}.
    * 
    * @param other the other {@code YoEnum} used to update this variable's value.
    * @throws ClassCastException if {@code other} cannot be casted as a {@code YoEnum}.
    */
   @Override
   public boolean setValue(YoVariable other, boolean notifyListeners)
   {
      @SuppressWarnings("unchecked")
      YoEnum<E> otherEnum = (YoEnum<E>) other;
      if (otherEnum.isBackedByEnum() && isBackedByEnum())
         return set(otherEnum.getEnumValue(), notifyListeners);
      else
         return set(otherEnum.getOrdinal(), notifyListeners);
   }

   /**
    * Gets the number of enum values declared for this {@code YoEnum}.
    *
    * @return number of constants for the enum backing this variable.
    */
   public int getEnumSize()
   {
      return enumValuesAsString.length;
   }

   @Override
   public String getValueAsString(String format)
   {
      return getStringValue();
   }

   @Override
   public boolean parseValue(String valueAsString, boolean notifyListeners)
   {
      if (valueAsString.toLowerCase().equals(NULL_VALUE_STRING))
      {
         return set(NULL_VALUE, notifyListeners);
      }

      for (int i = 0; i < enumValuesAsString.length; i++)
      {
         if (valueAsString.equals(enumValuesAsString[i]))
         {
            return set(i, notifyListeners);
         }
      }

      throw new IllegalArgumentException("Unable to parse value for parameter: " + getFullNameString() + ". String value: " + valueAsString);
   }

   @Override
   public String convertDoubleValueToString(String format, double value)
   {
      int ordinal = (int) Math.round(value);
      ordinal = Math.min(ordinal, getEnumSize() - 1);
      ordinal = Math.max(ordinal, allowNullValue ? NULL_VALUE : 0);
      if (ordinal == NULL_VALUE)
         return NULL_VALUE_STRING;
      else
         return enumValuesAsString[ordinal];
   }

   /**
    * Assesses if this YoEnum is equal to zero.
    *
    * @return boolean if this YoEnum's internal enum value is equal to null
    */
   @Override
   public boolean isZero()
   {
      return getEnumValue() == null;
   }

   /**
    * Creates a new YoEnum with the same parameters as this one, and registers it to the passed
    * {@link YoRegistry}.
    *
    * @param newRegistry YoRegistry to duplicate this YoEnum to
    * @return the newly created and registered YoEnum
    */
   @Override
   public YoEnum<E> duplicate(YoRegistry newRegistry)
   {
      YoEnum<E> duplicate;
      if (isBackedByEnum())
         duplicate = new YoEnum<>(getName(), getDescription(), newRegistry, getEnumType(), isNullAllowed());
      else
         duplicate = new YoEnum<>(getName(), getDescription(), newRegistry, isNullAllowed(), getEnumValuesAsString());
      duplicate.set(getOrdinal());
      return duplicate;
   }

   /**
    * Returns String representation of this YoEnum.
    *
    * @return String representing this YoEnum and its current value as an integer
    */
   @Override
   public String toString()
   {
      return String.format("%s: %s", getName(), getStringValue());
   }
}