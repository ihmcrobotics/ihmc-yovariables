package us.ihmc.yoVariables.parameters;

/**
 * Enumerates the possible states relating to initialization of a {@code YoParameter}.
 */
public enum ParameterLoadStatus
{
   /** The parameter has not been initialized and can not be used yet. */
   UNLOADED,
   /** The parameter has been initialized using its default value given at construction. */
   DEFAULT,
   /** The parameter has been initialized using external source such as a XML file. */
   LOADED;
}
