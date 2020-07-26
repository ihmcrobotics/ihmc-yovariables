package us.ihmc.yoVariables.euclid;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * {@code YoQuaternion} implementation which components {@code x}, {@code y}, {@code z}, {@code s}
 * are backed with {@code YoDouble}s.
 */
public class YoQuaternion implements QuaternionBasics
{
   private final String namePrefix;
   private final String nameSuffix;
   private final YoDouble x, y, z, s;

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
      namePrefix = YoGeometryNameTools.getCommonPrefix(qxVariable.getName(), qyVariable.getName(), qzVariable.getName(), qsVariable.getName());
      nameSuffix = YoGeometryNameTools.getCommonSuffix(qxVariable.getName(), qyVariable.getName(), qzVariable.getName(), qsVariable.getName());

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
   public YoQuaternion(String namePrefix, YoRegistry registry)
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
   public YoQuaternion(String namePrefix, String nameSuffix, YoRegistry registry)
   {
      this.namePrefix = namePrefix;
      this.nameSuffix = nameSuffix;

      x = new YoDouble(YoGeometryNameTools.createQxName(namePrefix, nameSuffix), registry);
      y = new YoDouble(YoGeometryNameTools.createQyName(namePrefix, nameSuffix), registry);
      z = new YoDouble(YoGeometryNameTools.createQzName(namePrefix, nameSuffix), registry);
      s = new YoDouble(YoGeometryNameTools.createQsName(namePrefix, nameSuffix), registry);
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

   /**
    * Gets the internal reference to the x-component used for this quaternion.
    *
    * @return the x-component as {@code YoVariable}.
    */
   public YoDouble getYoQx()
   {
      return x;
   }

   /**
    * Gets the internal reference to the y-component used for this quaternion.
    *
    * @return the y-component as {@code YoVariable}.
    */
   public YoDouble getYoQy()
   {
      return y;
   }

   /**
    * Gets the internal reference to the z-component used for this quaternion.
    *
    * @return the z-component as {@code YoVariable}.
    */
   public YoDouble getYoQz()
   {
      return z;
   }

   /**
    * Gets the internal reference to the s-component used for this quaternion.
    *
    * @return the s-component as {@code YoVariable}.
    */
   public YoDouble getYoQs()
   {
      return s;
   }

   /**
    * The name prefix used at creation of this {@code this}.
    *
    * @return the name prefix {@code String}.
    */
   public String getNamePrefix()
   {
      return namePrefix;
   }

   /**
    * The name suffix used at creation of this {@code this}.
    *
    * @return the name suffix {@code String}.
    */
   public String getNameSuffix()
   {
      return nameSuffix;
   }

   /**
    * Attaches a listener to {@code this} that is to be triggered when this quaternion components
    * change.
    *
    * @param variableChangedListener the listener to be attached.
    */
   public void attachVariableChangedListener(YoVariableChangedListener variableChangedListener)
   {
      x.addListener(variableChangedListener);
      y.addListener(variableChangedListener);
      z.addListener(variableChangedListener);
      s.addListener(variableChangedListener);
   }

   /**
    * Creates a copy of {@code this} by finding the duplicated {@code YoVariable}s in the given
    * {@link YoRegistry}.
    * <p>
    * This method does not duplicate {@code YoVariable}s. Assuming the given registry is a duplicate of
    * the registry that was used to create {@code this}, this method searches for the duplicated
    * {@code YoVariable}s and use them to duplicate {@code this}.
    * </p>
    *
    * @param newRegistry YoRegistry to duplicate {@code this} to.
    * @return the duplicate of {@code this}.
    */
   public YoQuaternion duplicate(YoRegistry newRegistry)
   {
      YoDouble x = (YoDouble) newRegistry.findVariable(getYoQx().getFullNameString());
      YoDouble y = (YoDouble) newRegistry.findVariable(getYoQy().getFullNameString());
      YoDouble z = (YoDouble) newRegistry.findVariable(getYoQz().getFullNameString());
      YoDouble s = (YoDouble) newRegistry.findVariable(getYoQs().getFullNameString());
      return new YoQuaternion(x, y, z, s);
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