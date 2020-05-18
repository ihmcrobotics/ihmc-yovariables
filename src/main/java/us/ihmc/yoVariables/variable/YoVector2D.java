package us.ihmc.yoVariables.variable;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoVector2D extends YoTuple2D implements Vector2DBasics, GeometryObject<YoVector2D>
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
   public YoVector2D(String namePrefix, YoVariableRegistry registry)
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
   public YoVector2D(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      super(namePrefix, nameSuffix, registry);
   }

   /** {@inheritDoc} */
   @Override
   public void set(YoVector2D other)
   {
      Vector2DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public boolean epsilonEquals(YoVector2D other, double epsilon)
   {
      return Vector2DBasics.super.epsilonEquals(other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public boolean geometricallyEquals(YoVector2D other, double epsilon)
   {
      return Vector2DBasics.super.geometricallyEquals(other, epsilon);
   }
}