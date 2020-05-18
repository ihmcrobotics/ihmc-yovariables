package us.ihmc.yoVariables.variable;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.util.YoFrameVariableNameTools;

public class YoQuaternion implements QuaternionBasics, GeometryObject<YoQuaternion>
{
   private final YoDouble x;
   private final YoDouble y;
   private final YoDouble z;
   private final YoDouble s;

   /**
    * Creates a new yo quaternion using the given variables.
    *
    * @param qxVariable an existing variable representing the x value of this {@code YoQuaternion}.
    * @param qyVariable an existing variable representing the y value of this {@code YoQuaternion}.
    * @param qzVariable an existing variable representing the z value of this {@code YoQuaternion}.
    * @param qsVariable an existing variable representing the z value of this {@code YoQuaternion}.
    */
   public YoQuaternion(YoDouble qxVariable, YoDouble qyVariable, YoDouble qzVariable, YoDouble qsVariable)
   {
      x = qxVariable;
      y = qyVariable;
      z = qzVariable;
      s = qsVariable;
   }

   /**
    * Creates a new yo quaternion, initializes its (x, y, z) components to zero and s to 1, and
    * registers variables to {@code registry}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoQuaternion(String namePrefix, YoVariableRegistry registry)
   {
      this(namePrefix, "", registry);
   }

   /**
    * Creates a new yo quaternion, initializes its (x, y, z) components to zero and s to 1, and
    * registers variables to {@code registry}.
    *
    * @param namePrefix a unique name string to use as the prefix for child variable names.
    * @param nameSuffix a string to use as the suffix for child variable names.
    * @param registry   the registry to register child variables to.
    */
   public YoQuaternion(String namePrefix, String nameSuffix, YoVariableRegistry registry)
   {
      x = new YoDouble(YoFrameVariableNameTools.createQxName(namePrefix, nameSuffix), registry);
      y = new YoDouble(YoFrameVariableNameTools.createQyName(namePrefix, nameSuffix), registry);
      z = new YoDouble(YoFrameVariableNameTools.createQzName(namePrefix, nameSuffix), registry);
      s = new YoDouble(YoFrameVariableNameTools.createQsName(namePrefix, nameSuffix), registry);
      s.set(1.0);
   }

   /** {@inheritDoc} */
   @Override
   public void setUnsafe(double qx, double qy, double qz, double qs)
   {
      x.set(qx);
      y.set(qy);
      z.set(qz);
      s.set(qs);
   }

   /** {@inheritDoc} */
   @Override
   public void set(YoQuaternion other)
   {
      QuaternionBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public double getX()
   {
      return x.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getY()
   {
      return y.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getZ()
   {
      return z.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getS()
   {
      return s.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public boolean epsilonEquals(YoQuaternion other, double epsilon)
   {
      return QuaternionBasics.super.epsilonEquals(other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public boolean geometricallyEquals(YoQuaternion other, double epsilon)
   {
      return QuaternionBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Tuple4DReadOnly)
         return equals((Tuple4DReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getTuple4DString(this);
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getX(), getY(), getZ(), getS());
   }
}