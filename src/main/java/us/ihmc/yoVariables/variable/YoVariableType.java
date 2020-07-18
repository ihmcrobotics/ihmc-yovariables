package us.ihmc.yoVariables.variable;

/**
 * Enumerates the primitives implemented as {@code YoVariable} and {@code YoParameter}.
 */
public enum YoVariableType
{
   /** Counterpart of {@code Double}: {@code YoDouble} and {@code DoubleParameter}. */
   DOUBLE,
   /** Counterpart of {@code Boolean}: {@code YoBoolean} and {@code BooleanParameter}. */
   BOOLEAN,
   /** Counterpart of {@code Enum}: {@code YoEnum} and {@code EnumParameter}. */
   ENUM,
   /** Counterpart of {@code Integer}: {@code YoInteger} and {@code IntegerParameter}. */
   INTEGER,
   /** Counterpart of {@code Long}: {@code YoLong} and {@code LongParameter}. */
   LONG;
}
