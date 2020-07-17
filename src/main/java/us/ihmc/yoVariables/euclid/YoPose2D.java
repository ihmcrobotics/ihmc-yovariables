package us.ihmc.yoVariables.euclid;

import us.ihmc.euclid.geometry.interfaces.Pose2DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.orientation.interfaces.Orientation2DBasics;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * {@code Pose2DBasics} implementation which position and orientation baked with
 * {@code YoVariable}s.
 */
public class YoPose2D implements Pose2DBasics, GeometryObject<YoPose2D>
{
   /** The position part of this pose 2D. */
   private final YoPoint2D position;
   /** The orientation part of this pose 2D. */
   private final YoOrientation2D orientation;

   /**
    * Creates a new {@code YoPose2D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoPose2D(String namePrefix, YoRegistry registry)
   {
      this(namePrefix, "", registry);
   }

   /**
    * Creates a new {@code YoPose2D}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoPose2D(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      position = new YoPoint2D(namePrefix, nameSuffix, registry);
      orientation = new YoOrientation2D(namePrefix, nameSuffix, registry);
   }

   @Override
   public void set(YoPose2D other)
   {
      Pose2DBasics.super.set(other);
   }

   @Override
   public Point2DBasics getPosition()
   {
      return position;
   }

   @Override
   public Orientation2DBasics getOrientation()
   {
      return orientation;
   }

   @Override
   public void setX(double x)
   {
      position.setX(x);
   }

   @Override
   public void setY(double y)
   {
      position.setY(y);
   }

   @Override
   public void setYaw(double yaw)
   {
      orientation.setYaw(yaw);
   }

   @Override
   public double getX()
   {
      return position.getX();
   }

   @Override
   public double getY()
   {
      return position.getY();
   }

   @Override
   public double getYaw()
   {
      return orientation.getYaw();
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Pose2DReadOnly)
         return equals((Pose2DReadOnly) object);
      else
         return false;
   }

   @Override
   public boolean epsilonEquals(YoPose2D other, double epsilon)
   {
      return Pose2DBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(YoPose2D other, double epsilon)
   {
      return Pose2DBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getPose2DString(this);
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(position, orientation);
   }
}