package us.ihmc.yoVariables.variable;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoVector3D extends YoTuple3D implements Vector3DBasics, GeometryObject<YoVector3D>
{
   /**
    * Creates a new yo vector using the given variables.
    *
    * @param xVariable the x-component variable.
    * @param yVariable the y-component variable.
    * @param zVariable the z-component variable.
    */
   public YoVector3D(YoDouble xVariable, YoDouble yVariable, YoDouble zVariable)
   {
      super(xVariable, yVariable, zVariable);
   }

   /**
    * Creates a new yo vector, initializes its components to zero, and registers variables to
    * {@code registry}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoVector3D(String namePrefix, YoRegistry registry)
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
   public YoVector3D(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      super(namePrefix, nameSuffix, registry);
   }

   /** {@inheritDoc} */
   @Override
   public boolean epsilonEquals(YoVector3D other, double epsilon)
   {
      return Vector3DBasics.super.epsilonEquals(other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public boolean geometricallyEquals(YoVector3D other, double epsilon)
   {
      return Vector3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public void set(YoVector3D other)
   {
      Vector3DBasics.super.set(other);
   }
}