package us.ihmc.yoVariables.euclid;

import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * {@code Point3DBasics} implementation which coordinates {@code x}, {@code y}, {@code z} are backed
 * with {@code YoDouble}s.
 */
public class YoPoint3D extends YoTuple3D implements Point3DBasics
{
   /**
    * Creates a new yo point using the given variables.
    *
    * @param xVariable the x-coordinate variable.
    * @param yVariable the y-coordinate variable.
    * @param zVariable the z-coordinate variable.
    */
   public YoPoint3D(YoDouble xVariable, YoDouble yVariable, YoDouble zVariable)
   {
      super(xVariable, yVariable, zVariable);
   }

   /**
    * Creates a new yo point, initializes its coordinates to zero, and registers variables to
    * {@code registry}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoPoint3D(String namePrefix, YoRegistry registry)
   {
      super(namePrefix, registry);
   }

   /**
    * Creates a new yo point, initializes its coordinates to zero, and registers variables to
    * {@code registry}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoPoint3D(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      super(namePrefix, nameSuffix, registry);
   }
}