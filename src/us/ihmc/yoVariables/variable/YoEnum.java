package us.ihmc.yoVariables.variable;

import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.Arrays;

/**
 * Enum implementation of the YoVariable class.
 *
 * <p>All abstract functions of YoVariable will be implemented using enum type for interpretation.
 * Values will be interpreted, compared, and returned as enums rather than any native types.
 */
public class YoEnum<T extends Enum<T>> extends YoVariable<YoEnum<T>>
{
   public static final int NULL_VALUE = -1;

   private Class<T> enumType;
   private final boolean allowNullValue;
   private T[] enumValues;
   private String[] enumValuesAsString;

   private int valueOrdinal;

   /**
    * Create a new YoEnum. This will call {@link YoVariable(YoVariableType, String, String, YoVariableRegistry)} with
    * {@link YoVariableType#ENUM} and the given values.
    *
    * @param name String uniquely identifying this YoEnum
    * @param description String describing this YoEnum's purpose
    * @param registry YoVariableRegistry for this YoEnum to register itself to after initialization
    * @param enumType the class representing the type of the enum
    * @param allowNullValue boolean determining if null enum values are permitted
    */
   public YoEnum(String name, String description, YoVariableRegistry registry, Class<T> enumType, boolean allowNullValue)
   {
      super(YoVariableType.ENUM, name, description, registry);

      this.enumType = enumType;
      this.allowNullValue = allowNullValue;
      this.enumValues = enumType.getEnumConstants();

      enumValuesAsString = new String[enumValues.length];
      for (int i = 0; i < enumValues.length; i++)
      {
         enumValuesAsString[i] = enumValues[i].toString();
      }

      set(0, true);
   }

   /**
    * Create a new YoEnum. This will call {@link YoEnum(String, YoVariableRegistry, Class)} with the given values.
    *
    * @param name String uniquely identifying this YoEnum
    * @param registry YoVariableRegistry for this YoEnum to register itself to after initialization
    * @param enumType the class representing the type of the enum
    */
   public static <T extends Enum<T>> YoEnum<T> create(String name, Class<T> enumType, YoVariableRegistry registry)
   {
      return new YoEnum<>(name, registry, enumType);
   }

   /**
    * Create a new YoEnum. This will call {@link YoEnum(String, String, YoVariableRegistry, Class, Boolean)} with the given values.
    *
    * @param name String uniquely identifying this YoEnum
    * @param description String describing this YoEnum's purpose
    * @param registry YoVariableRegistry for this YoEnum to register itself to after initialization
    * @param enumType the class representing the type of the enum
    * @param allowNullValue boolean determining if null enum values are permitted
    */
   public static <T extends Enum<T>> YoEnum<T> create(String name, String description, Class<T> enumType, YoVariableRegistry registry, boolean allowNullValue)
   {
      return new YoEnum<>(name, description, registry, enumType, allowNullValue);
   }

   /**
    * Create a new YoEnum based on Strings.
    *
    * <p>When a new YoEnum has to be created based on data from an external source, this constructor can be used.
    * The main use case is the LogVisualizer and the SCSVisualizer code. Do not use this constructor in controls code.
    *
    * <p>The functions working on Enums directly will throw UnsupportedOperationException when this constructor is used.
    *
    * @param name String uniquely identifying this YoEnum
    * @param description String describing this YoEnum's purpose
    * @param registry YoVariableRegistry for this YoEnum to register itself to after initialization
    * @param values String array of values that this enum can take
    */
   public YoEnum(String name, String description, YoVariableRegistry registry, boolean allowNullValues, String... values)
   {
      super(YoVariableType.ENUM, name, description, registry);

      this.enumType = null;
      this.allowNullValue = allowNullValues;
      this.enumValues = null;

      enumValuesAsString = Arrays.copyOf(values, values.length);
      set(0, true);
   }

   /**
    * Create a new YoEnum. This will call {@link #YoEnum(String, String, YoVariableRegistry, Class, boolean)} with the given values,
    * an empty description and false for allowNullValue.
    *
    * @param name String uniquely identifying this YoEnum
    * @param registry YoVariableRegistry for this YoEnum to register itself to after initialization
    * @param enumType the class representing the type of the enum
    */
   public YoEnum(String name, YoVariableRegistry registry, Class<T> enumType)
   {
      this(name, "", registry, enumType, false);
   }

   /**
    * Create a new YoEnum. This will call {@link #YoEnum(String, String, YoVariableRegistry, Class, boolean)} with the given values
    * and an empty description.
    *
    * @param name String uniquely identifying this YoEnum
    * @param registry YoVariableRegistry for this YoEnum to register itself to after initialization
    * @param enumType the class representing the type of the enum
    * @param allowNullValue boolean determining if null enum values are permitted
    */
   public YoEnum(String name, YoVariableRegistry registry, Class<T> enumType, boolean allowNullValue)
   {
      this(name, "", registry, enumType, allowNullValue);
   }

   /**
    * Assesses if this YoEnum is backed by a Class as an enumType.
    *
    * @return if not based on a Class as an enumType
    * @see #enumType
    * @see YoEnum(String, String, YoVariableRegistry, boolean, String...)
    */
   public boolean isBackedByEnum()
   {
      return enumType != null;
   }

   /**
    * Essentially performs {@link #isBackedByEnum()} and throws an exception if false.
    *
    * @throws UnsupportedOperationException if not backed by a Class as an enumType
    */
   private void checkIfBackedByEnum()
   {
      if (enumType == null)
      {
         throw new UnsupportedOperationException("This YoEnum is not backed by an Enum variable.");
      }
   }

   /**
    * Check if the value contained by this variable is equal to the given enum.
    *
    * @param value Enum to be compared
    * @return boolean if the given enum value and this YoEnum's value are the same, or both null
    * @throws UnsupportedOperationException from {@link #checkIfBackedByEnum()}
    */
   public boolean valueEquals(T value)
   {
      checkIfBackedByEnum();
      if (valueOrdinal == NULL_VALUE)
         return value == null;

      return (value.ordinal() == valueOrdinal);
   }

   /**
    * Retrieves this YoEnum's enumType if it is backed by an enum.
    *
    * @return enumType
    * @throws UnsupportedOperationException from {@link #checkIfBackedByEnum()}
    */
   public Class<T> getEnumType()
   {
      checkIfBackedByEnum();
      return enumType;
   }

   public void setEnumType(Class<T> enumType)
   {
      this.enumType = enumType;
      enumValues = enumType.getEnumConstants();
      enumValuesAsString = new String[enumValues.length];
      for (int i = 0; i < enumValues.length; i++)
      {
         enumValuesAsString[i] = enumValues[i].toString();
      }
   }

   /**
    * Calls {@link #set(T, boolean)} with the given value and true.
    *
    * @param enumValue enum to set this YoEnum's internal enum state to
    */
   public void set(T enumValue)
   {
      set(enumValue, true);
   }

   /**
    * Calls {@link #set(int, boolean)} with the given value and true.
    *
    * @param ordinal integer enum ordinal to set this YoEnum's internal enum state to
    */
   public void set(int ordinal)
   {
      set(ordinal, true);
   }

   /**
    * Sets the YoEnum to the given enum.
    *
    * @param enumValue enum to set this YoEnum's internal enum state to
    * @param notifyListeners boolean determining whether or not to call {@link #notifyVariableChangedListeners()}
    * @return boolean if given value is valid and YoEnum is set or the same
    * @throws RuntimeException if enumValue is null and null values are disallowed for this YoEnum
    * @throws RuntimeException if enumValue ordinal falls out of allowed range for this YoEnum
    */
   public boolean set(T enumValue, boolean notifyListeners)
   {
      checkIfBackedByEnum();
      if (!allowNullValue && (enumValue == null))
      {
         throw new RuntimeException(
               "Setting YoEnum " + getName() + " to null. Must set allowNullValue to true in the constructor if you ever want to set it to null.");
      }

      return set(enumValue == null ? NULL_VALUE : enumValue.ordinal(), notifyListeners);
   }

   /**
    * Sets the YoEnum to the given ordinal.
    *
    * @param ordinal integer ordinal for the enum value to set this YoEnum to
    * @param notifyListeners boolean determining whether or not to call {@link #notifyVariableChangedListeners()}
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
               notifyVariableChangedListeners();
            }
            return true;
         }
      }
      else
      {
         valueOrdinal = ordinal;
         if (notifyListeners)
         {
            notifyVariableChangedListeners();
         }
         return true;
      }

      return false;

   }

   /**
    * Exception-throwing check if the given value falls within the allowed range for this YoEnum.
    *
    * @param ordinal integer ordinal value to check
    * @throws RuntimeException if the given value is null and null is not allowed for this YoEnum or if the value falls out of the allowed range
    */
   private void checkBounds(int ordinal)
   {
      if ((ordinal < 0 && !(allowNullValue && ordinal == NULL_VALUE)) || ordinal >= enumValuesAsString.length)
      {
         throw new RuntimeException("Enum constant associated with value " + ordinal + " not present. VariableName = " + this.getFullNameWithNameSpace());
      }
   }

   /**
    * Retrieves {@link #allowNullValue}.
    *
    * @return boolean if null values are allowed for this YoEnum
    */
   public boolean getAllowNullValue()
   {
      return allowNullValue;
   }

   /**
    * Retrieves {@link #enumValues}.
    *
    * @return list of possible enum values for this YoEnum
    * @throws UnsupportedOperationException if this YoEnum is not backed by a Class for enumType
    */
   public T[] getEnumValues()
   {
      checkIfBackedByEnum();
      return enumValues;
   }

   /**
    * Retrieves {@link #enumValuesAsString}.
    *
    * @return String representation of list of enum values for this YoEnum
    */
   public String[] getEnumValuesAsString()
   {
      return enumValuesAsString;
   }

   /**
    * Retrieve the enum value of this YoEnum.
    *
    * @return this YoEnum's internal enum value
    * @throws UnsupportedOperationException if this YoEnum is not backed by a Class for enumType
    */
   public T getEnumValue()
   {
      checkIfBackedByEnum();
      return valueOrdinal == NULL_VALUE ? null : enumValues[valueOrdinal];
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
         return "null";
      }
      else
      {
         return enumValuesAsString[valueOrdinal];
      }
   }

   /**
    * Set the value of this YoEnum using the given double, interpreted as a rounded integer ordinal for an enum value.
    *
    * @param value double to convert and set this YoEnum to
    * @param notifyListeners boolean determining whether or not to call {@link #notifyVariableChangedListeners()}
    */
   @Override public void setValueFromDouble(double value, boolean notifyListeners)
   {
      try
      {
         int index = (int) Math.round(value);
         set(index, notifyListeners);
      }
      catch (RuntimeException ignored)
      {

      }
   }

   /**
    * Retrieves this YoEnum's value as a double.
    *
    * @return return-casted double value of this YoEnum's internal enum value
    */
   @Override public double getValueAsDouble()
   {
      return valueOrdinal;
   }

   /**
    * Returns String representation of this YoEnum.
    *
    * @return String representing this YoEnum and its current value as an integer
    */
   @Override public String toString()
   {
      return String.format("%s: %s", getName(), getStringValue());
   }

   /**
    * Appends the value of this YoEnum to the end of the given StringBuffer.
    *
    * @param stringBuffer StringBuffer to which the value will be appended
    */
   @Override public void getValueString(StringBuffer stringBuffer)
   {
      stringBuffer.append(getStringValue());
   }

   /**
    * Appends the YoEnum representation of the given double value to the given StringBuffer.
    *
    * @param stringBuffer StringBuffer to append to
    * @param doubleValue double value to convert to YoEnum representation
    */
   @Override public void getValueStringFromDouble(StringBuffer stringBuffer, double doubleValue)
   {
      int index = (int) Math.round(doubleValue);
      checkBounds(index);
      if (index == NULL_VALUE)
      {
         stringBuffer.append("null");
      }
      else
      {
         stringBuffer.append(enumValuesAsString[index]);
      }
   }

   /**
    * Retrieves this YoEnum's value as a long.
    *
    * @return return-casted long value of this YoEnum's internal enum value
    */
   @Override public long getValueAsLongBits()
   {
      return valueOrdinal;
   }

   /**
    * Sets the internal enum value of this YoEnum using the static integer enum ordinal cast of the passed long value.
    *
    * @param value long to set this variable's value to
    * @param notifyListeners boolean determining whether or not to call {@link #notifyVariableChangedListeners()}
    */
   @Override public void setValueFromLongBits(long value, boolean notifyListeners)
   {
      set((int) value, notifyListeners);
   }

   /**
    * Creates a new YoEnum with the same parameters as this one, and registers it to the passed {@link YoVariableRegistry}.
    *
    * @param newRegistry YoVariableRegistry to duplicate this YoEnum to
    * @return the newly created and registered YoEnum
    */
   @Override public YoEnum<T> duplicate(YoVariableRegistry newRegistry)
   {
      YoEnum<T> retVar = new YoEnum<>(getName(), getDescription(), newRegistry, getEnumType(), getAllowNullValue());
      retVar.set(getEnumValue());
      return retVar;
   }

   /**
    * Sets the internal value of this YoEnum to the current value of the passed YoEnum.
    *
    * @param value YoEnum value to set this variable's value to
    * @param notifyListeners boolean determining whether or not to call {@link #notifyVariableChangedListeners()}
    * @return boolean whether or not internal state differed from the passed value
    */
   @Override public boolean setValue(YoEnum<T> value, boolean notifyListeners)
   {
      return set(value.getEnumValue(), notifyListeners);
   }

   /**
    * Retrieves {@link #valueOrdinal}.
    *
    * @return integer enum ordinal of this YoEnum's internal enum state
    */
   public int getOrdinal()
   {
      return valueOrdinal;
   }

   /**
    * Assesses the number of enum values declared for this YoEnum.
    *
    * @return length {@link #enumValuesAsString}
    */
   public int getEnumSize()
   {
      return enumValuesAsString.length;
   }

   /**
    * Assesses if this YoEnum is equal to zero.
    *
    * @return boolean if this YoEnum's internal enum value is equal to null
    */
   @Override public boolean isZero()
   {
      return getEnumValue() == null;
   }
}