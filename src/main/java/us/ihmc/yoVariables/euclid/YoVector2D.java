package us.ihmc.yoVariables.euclid;

import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * {@code YoVector2D} implementation which components {@code x}, {@code y} are backed with
 * {@code YoDouble}s.
 */
public class YoVector2D extends YoTuple2D implements Vector2DBasics
{
   /**
    * Creates a new yo vector using the given variables.
    *
    * @param xVariable the x-component variable.
    * @param yVariable the y-component variable.
    */
   public YoVector2D(YoDouble xVariable, YoDouble yVariable)
   {
      super(xVariable, yVariable);
   }

   /**
    * Creates a new yo vector, initializes its components to zero, and registers variables to
    * {@code registry}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoVector2D(String namePrefix, YoRegistry registry)
   {
      super(namePrefix, registry);
   }

   /**
    * Creates a new yo vector, initializes its components to zero, and registers variables to
    * {@code registry}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoVector2D(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      super(namePrefix, nameSuffix, registry);
   }
}