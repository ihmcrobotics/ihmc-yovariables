package us.ihmc.yoVariables.euclid;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.orientation.Orientation2D;
import us.ihmc.euclid.orientation.interfaces.Orientation2DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoFrameVariableNameTools;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * {@code Orientation2DBasics} implementation which position and orientation baked with
 * {@code YoVariable}s.
 */
public class YoOrientation2D implements Orientation2DBasics, GeometryObject<YoOrientation2D>
{
   /** The angle in radians about the z-axis. */
   private final YoDouble yaw;

   /** Orientation used to transform {@code this} in {@link #applyTransform(Transform)}. */
   private final Orientation2D orientation = new Orientation2D();

   /**
    * Creates a new {@code YoOrientation2D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoOrientation2D(String namePrefix, YoRegistry registry)
   {
      this(namePrefix, "", registry);
   }

   /**
    * Creates a new {@code YoOrientation2D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoOrientation2D(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      yaw = new YoDouble(YoFrameVariableNameTools.createName(namePrefix, "yaw", nameSuffix), registry);
   }

   /** {@inheritDoc} */
   @Override
   public void set(YoOrientation2D other)
   {
      Orientation2DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setYaw(double yaw)
   {
      this.yaw.set(yaw);
   }

   /** {@inheritDoc} */
   @Override
   public double getYaw()
   {
      return yaw.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public boolean epsilonEquals(YoOrientation2D other, double epsilon)
   {
      return Orientation2DBasics.super.epsilonEquals(other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public boolean geometricallyEquals(YoOrientation2D other, double epsilon)
   {
      return Orientation2DBasics.super.geometricallyEquals(other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public void applyTransform(Transform transform)
   {
      orientation.set(this);
      orientation.applyTransform(transform);
      set(orientation);
   }

   /** {@inheritDoc} */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      orientation.set(this);
      orientation.applyInverseTransform(transform);
      set(orientation);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Orientation2DReadOnly)
         return equals((Orientation2DReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getOrientation2DString(this);
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getYaw());
   }
}